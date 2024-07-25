import rospy
import json

from pydoc import locate
from base.ros1.tools import convert_ros_message_to_dictionary
from base.subscriber import BridgeSubscriber
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1


class BridgeSubscriberRos1(BridgeSubscriber):
    def __init__(self, bridge: "ProBridgeRos1", clients, settings) -> None:
        self.msg_qos = settings.get("qos")
        super().__init__(bridge=bridge, clients=clients, settings=settings)

    def create_sub(self, bridge: "ProBridgeRos1", settings: dict):
        if settings["name"] == "/tf_static":
            rospy.loginfo("created static tf listener")
            rospy.Timer(rospy.Duration(0.5), self.listen_tf_static)
        else:
            rospy.Subscriber(name=settings["name"], data_class=locate(settings["type"]), callback=self.msg_callback, queue_size=1)

        rospy.loginfo("Create publisher in topic: " + settings["name"])

    def listen_tf_static(self, event):
        from tf2_msgs.msg import TFMessage
        try:
            ros_msg = rospy.wait_for_message("/tf_static", topic_type=TFMessage)
        except:
            return
        self.msg_callback(ros_msg)

    def prepare_msg(self, m_type, m_name, m_qos, m_data):
        d = self.convert_msg(m_data)
        j_data = {
            "v": 1,
            "t": m_type,
            "n": m_name,
            "d": d,
        }
        if m_qos is not None:
            j_data["q"] = m_qos  # Allow to publish messages from ros1 to ros2 with specified QOS

        return json.dumps(j_data)

    def getrostime(self) -> float:
        return rospy.get_rostime().to_sec()

    def convert_msg(self, m_data):
        return convert_ros_message_to_dictionary(m_data)

    def is_latch_msg(self, msg) -> bool:
        if self.is_latch is not None:
            return self.is_latch
        return msg._connection_header["latching"] == "1"
