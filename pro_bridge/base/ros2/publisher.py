from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields
from pydoc import locate
from base.publisher import BridgePublisher
from base.ros2.tools import get_qos
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros2.node import ProBridgeRos2


class BridgePublisherRos2(BridgePublisher):
    bridge: "ProBridgeRos2"

    def __init__(self, bridge: "ProBridgeRos2") -> None:
        super().__init__(bridge)

    def create_pub(self, b_msg: dict) -> Publisher:
        qos_profile = get_qos(b_msg.get("q", 0))
        publisher = self.bridge.create_publisher(msg_type=locate(b_msg["t"]), topic=b_msg["n"], qos_profile=qos_profile)
        self.publishers.update({b_msg["n"]: publisher})
        self.bridge.loginfo('Subscribed on topic "' + b_msg["n"] + '".')
        return publisher

    def dict2ros(self, msg):
        ros_msg = locate(msg["t"])()  # type: ignore
        set_message_fields(ros_msg, msg["d"])
        return ros_msg

    def override_msg(self, msg: dict):
        def override(msg: dict):
            if not isinstance(msg, dict):
                return
            for key, value in msg.items():
                if key == "header" and msg["header"].get("seq") is not None:
                    msg["header"].pop("seq")
                    msg["header"]["stamp"]["sec"] = msg["header"]["stamp"].pop("secs")
                    msg["header"]["stamp"]["nanosec"] = msg["header"]["stamp"].pop("nsecs")
                elif key == "clock" and msg["clock"].get("secs") is not None:
                    msg["clock"]["sec"] = msg["clock"].pop("secs")
                    msg["clock"]["nanosec"] = msg["clock"].pop("nsecs")

                elif isinstance(value, list):
                    for v in msg[key]:
                        override(v)
            return msg

        if msg["t"] == "tf.msg.tfMessage":
            msg["t"] = "tf2_msgs.msg.TFMessage"

        override(msg["d"])
