import rospy

from base.ros1.tools import convert_dictionary_to_ros_message
from pydoc import locate
from base.publisher import BridgePublisher
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1


class BridgePublisherRos1(BridgePublisher):
    bridge: "ProBridgeRos1"

    def __init__(self, bridge: "ProBridgeRos1") -> None:
        super().__init__(bridge)

    def create_pub(self, b_msg: dict) -> rospy.Publisher:
        publisher = rospy.Publisher(name=b_msg["n"], data_class=locate(b_msg["t"]), queue_size=1)
        self.publishers.update({b_msg["n"]: publisher})
        rospy.loginfo('Subscribed on topic "' + b_msg["n"] + '".')
        return publisher

    def dict2ros(self, msg):
        return convert_dictionary_to_ros_message(locate(msg["t"])._type, msg["d"])  # type: ignore

    def override_msg(self, msg: dict):
        def override(msg: dict):
            if not isinstance(msg, dict):
                return
            for key, value in msg.items():
                if key == "header" and msg["header"]["stamp"].get("sec") is not None:
                    msg["header"]["stamp"]["secs"] = msg["header"]["stamp"].pop("sec")
                    msg["header"]["stamp"]["nsecs"] = msg["header"]["stamp"].pop("nanosec")
                elif key == "clock" and msg["clock"].get("sec") is not None:
                    msg["clock"]["secs"] = msg["clock"].pop("sec")
                    msg["clock"]["nsecs"] = msg["clock"].pop("nsanosec")

                elif isinstance(value, list):
                    for v in msg[key]:
                        override(v)
            return msg

        override(msg["d"])
