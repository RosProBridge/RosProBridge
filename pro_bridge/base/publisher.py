import os
import json
import gzip

from abc import ABC, abstractmethod
from typing import Union
from typing import TYPE_CHECKING
from base.server import BridgeServerTCP, BridgeServerUDP

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1
    from base.ros2.node import ProBridgeRos2


class BridgePublisher(ABC):
    def __init__(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"]) -> None:
        self.__ignorable_topics = []
        self.bridge = bridge
        self.publishers = {}
        self.tcp_server = BridgeServerTCP(bridge.host, self.on_msg)
        self.udp_server = BridgeServerUDP(bridge.host, self.on_msg)

    @abstractmethod
    def create_pub(self, b_msg: dict):
        """Create ROS publisher"""

    @abstractmethod
    def override_msg(self, msg: dict):
        """Override message from ROS2 to ROS1 or from ROS1 to ROS2 based on env('ROS_VERSION')"""

    @abstractmethod
    def dict2ros(self, msg):
        """Convert dictionary to ros message"""

    def on_msg(self, msg: bytes):
        """
        Decode and publish message to ROS
        """
        try:
            # parse bridge msg
            message = gzip.decompress(msg).decode("ascii").rstrip()
            b_msg = json.loads(message)
        except Exception as e:
            self.bridge.logwarn("Invalid received message. Failed to parse: {}".format(str(e)))
            return

        # allow to recive messages from ROS2 in ROS1 | from ROS1 in ROS2
        if b_msg.get("v") != os.environ["ROS_VERSION"]:
            try:
                self.override_msg(b_msg)
            except Exception as e:
                self.bridge.logwarn("Invalid received message. Failed to override message: {}".format(str(e)))
                return

        if b_msg["n"] in self.__ignorable_topics:
            return

        publisher = self.publishers.get(b_msg["n"])
        if publisher is None:
            try:
                publisher = self.create_pub(b_msg)
            except Exception as e:
                self.bridge.logerr(
                    "Failed to create a bridge publisher to the topic: {}. Make sure the workspace with messages is sourced. Added to ignore list".format(
                        b_msg.get("n", "")
                    )
                )
                self.__ignorable_topics.append(b_msg.get("n"))
                return

        try:
            ros_msg = self.dict2ros(b_msg)
        except:
            self.bridge.logerr(
                "Failed to convert dictionary to ROS message. Make sure the message is correct. Type: {}. Added to ignore list".format(
                    b_msg.get("t", "")
                )
            )
            self.__ignorable_topics.append(b_msg.get("n"))
            return

        publisher.publish(ros_msg)  # type: ignore

    def Stop(self):
        self.tcp_server.Stop()
        self.udp_server.Stop()
