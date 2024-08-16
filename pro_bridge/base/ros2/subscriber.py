import json
import gzip

from rclpy.serialization import serialize_message
from base.subscriber import BridgeSubscriber
from pydoc import locate
from rosidl_runtime_py import message_to_ordereddict
from base.ros2.tools import get_qos
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros2.node import ProBridgeRos2


class BridgeSubscriberRos2(BridgeSubscriber):
    def __init__(self, bridge: "ProBridgeRos2", tcp_clients, settings) -> None:
        self.msg_qos = settings.get("qos")
        super().__init__(bridge=bridge, tcp_clients=tcp_clients, settings=settings)

    def create_sub(self, bridge: "ProBridgeRos2", settings: dict):
        if self.msg_qos is None:
            self.msg_qos = "qos_profile_system_default"
            self.bridge.logwarn(
                "QOS Profile was not set in config for topic {}. Set default 'qos_profile_system_default'".format(settings.get("name", ""))
            )

        qos_profile = get_qos(self.msg_qos)
        bridge.create_subscription(
            msg_type=locate(settings["type"]), topic=settings["name"], callback=self.msg_callback, qos_profile=qos_profile
        )
        bridge.get_logger().info("Create publisher in topic: " + settings["name"])

    def prepare_msg(self, m_type, m_name, m_qos, m_data):
        json_compressed = gzip.compress(
            json.dumps({"v": 2, "t": m_type, "n": m_name, "q": m_qos, "c": self.compression_level}).encode('utf-8'),
            compresslevel=1
        )
        serialized_message = serialize_message(message=m_data)
        if self.compression_level > 0:
            serialized_message = gzip.compress(serialized_message, compresslevel=self.compression_level)

        json_length = len(json_compressed).to_bytes(length=2, byteorder='little')
        return json_length + json_compressed + serialized_message

    def getrostime(self) -> float:
        return self.bridge.get_clock().now().nanoseconds / 1e9  # type: ignore

    def is_latch_msg(self, msg) -> bool:
        if self.is_latch is not None:
            return self.is_latch
        return False
