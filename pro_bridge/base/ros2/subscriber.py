import json

from base.subscriber import BridgeSubscriber
from pydoc import locate
from rosidl_runtime_py import message_to_ordereddict
from base.ros2.tools import get_qos
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros2.node import ProBridgeRos2


class BridgeSubscriberRos2(BridgeSubscriber):
    def __init__(self, bridge: "ProBridgeRos2", clients, settings) -> None:
        self.msg_qos = settings.get("qos")
        super().__init__(bridge=bridge, clients=clients, settings=settings)

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
        d = self.convert_msg(m_data)
        j = json.dumps({"v": 2, "t": m_type, "n": m_name, "q": m_qos, "d": d})
        return j

    def getrostime(self) -> float:
        return self.bridge.get_clock().now().nanoseconds / 1e9  # type: ignore

    def convert_msg(self, m_data):
        return message_to_ordereddict(m_data)

    def is_latch_msg(self, msg) -> bool:
        if self.is_latch is not None:
            return self.is_latch
        return False
