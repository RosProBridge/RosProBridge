import os
import json
import gzip

from abc import ABC, abstractmethod
from typing import Union
from typing import TYPE_CHECKING
from base.server import BridgeServerTCP

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1
    from base.ros2.node import ProBridgeRos2


class BridgePublisher(ABC):
    def __init__(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"]) -> None:
        self.__ignorable_topics = []
        self.bridge = bridge
        self.publishers = {}
        self.tcp_server = BridgeServerTCP(bridge.host, self.on_msg)

    @abstractmethod
    def create_pub(self, b_msg: dict):
        """Create ROS publisher"""

    @abstractmethod
    def override_msg(self, msg_type: str, msg_packet: bytes):
        """Override message from ROS2 to ROS1 or from ROS1 to ROS2 based on env('ROS_VERSION')"""

    @abstractmethod
    def deserialize_ros_message(self, binary_msg: bytes, msg_type: str):
        """Deserialize ROS message"""

    def on_msg(self, msg: bytes):
        """
        Decode and publish message to ROS, or demux service request/response packets.
        """
        try:
            json_length = int.from_bytes(msg[:2], byteorder="little")
            json_data: dict = json.loads(gzip.decompress(msg[2 : 2 + json_length]))
        except Exception as e:
            self.bridge.logwarn("Invalid received message. Failed to parse: {}".format(str(e)))
            return

        kind = json_data.get("k")
        name = json_data.get("n", "")

        compression_level = json_data.get("c")
        if (compression_level is None) or (not isinstance(compression_level, int)):
            if kind in (1, 2):
                self.bridge.logerr(
                    "Compression level field was not filled for the service packet {}. Dropping".format(name)
                )
                return
            self.bridge.logerr(
                "Compression level field was not filled for the topic {}. Added to ignore list".format(name)
            )
            self.__ignorable_topics.append(name)
            return

        if compression_level > 0:
            binary_packet = gzip.decompress(msg[2 + json_length :])
        else:
            binary_packet = msg[2 + json_length :]

        if kind == 1:
            handler = getattr(self.bridge, "handle_service_request", None)
            if handler is None:
                self.bridge.logwarn("Received service request but this bridge does not support services")
                return
            handler(json_data, binary_packet)
            return

        if kind == 2:
            handler = getattr(self.bridge, "handle_service_response", None)
            if handler is None:
                self.bridge.logwarn("Received service response but this bridge does not support services")
                return
            handler(json_data, binary_packet)
            return

        topic_name = name

        if topic_name in self.__ignorable_topics:
            return

        # allow to recive messages from ROS2 in ROS1 | from ROS1 in ROS2
        if str(json_data.get("v")) != os.environ["ROS_VERSION"]:
            try:
                binary_packet = self.override_msg(msg_type=json_data.get("t"), msg_packet=binary_packet)
            except Exception as e:
                self.bridge.logerr("Failed to override message {}".format(str(e)))
                return

        publisher = self.publishers.get(topic_name)
        if publisher is None:
            try:
                publisher = self.create_pub(json_data)
            except Exception as e:
                self.bridge.logerr(
                    "Failed to create a bridge publisher to the topic: {}. Make sure the workspace with messages is sourced. Added to ignore list".format(
                        json_data.get("n", "")
                    )
                )
                self.__ignorable_topics.append(topic_name)
                return

        try:
            ros_msg = self.deserialize_ros_message(binary_packet, json_data.get("t"))
        except:
            self.bridge.logerr(
                "Failed to deserialize to ROS message. Make sure the message is correct. Type: {}. Added to ignore list".format(
                    json_data.get("t", "")
                )
            )
            self.__ignorable_topics.append(topic_name)
            return

        publisher.publish(ros_msg)  # type: ignore

    def Stop(self):
        self.tcp_server.Stop()
