import rospy
import json
import gzip

from io import BytesIO
from pydoc import locate
from base.ros1.tools import convert_ros_message_to_dictionary
from base.subscriber import BridgeSubscriber
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1


class BridgeSubscriberRos1(BridgeSubscriber):
    def __init__(self, bridge: "ProBridgeRos1", tcp_clients, settings) -> None:
        self.msg_qos = settings.get("qos")
        super().__init__(bridge=bridge, tcp_clients=tcp_clients, settings=settings)

    def create_sub(self, bridge: "ProBridgeRos1", settings: dict):
        if settings["name"] == "/tf_static":
            rospy.loginfo("created static tf listener")
            rospy.Timer(rospy.Duration(0.5), self.listen_tf_static)
        else:
            rospy.Subscriber(name=settings["name"], data_class=locate(settings["type"]), callback=self.msg_callback, queue_size=1)

        rospy.loginfo("Create publisher in topic: " + settings["name"])

    def listen_tf_static(self, event):
        # TODO: Should be deleted
        from tf2_msgs.msg import TFMessage
        try:
            ros_msg = rospy.wait_for_message("/tf_static", topic_type=TFMessage)
        except:
            return
        self.msg_callback(ros_msg)

    def prepare_msg(self, m_type, m_name, m_qos, m_data):
        j_data = {
            "v": 1,
            "t": m_type,
            "n": m_name,
            "c": self.compression_level
        }
        if m_qos is not None:
            j_data["q"] = m_qos  # Allow to publish messages from ros1 to ros2 with specified QOS

        json_compressed = gzip.compress(
            json.dumps(j_data).encode('utf-8'),
            compresslevel=1
        )
        buffer = BytesIO()
        m_data.serialize(buffer)

        if self.compression_level >0:
            serialized_message = gzip.compress(buffer.getvalue(), compresslevel=self.compression_level)
        else:
            serialized_message = buffer.getvalue()

        json_length = len(json_compressed).to_bytes(length=2, byteorder='little')
        return json_length + json_compressed + serialized_message

    def getrostime(self) -> float:
        return rospy.get_rostime().to_sec()


    def is_latch_msg(self, msg) -> bool:
        if self.is_latch is not None:
            return self.is_latch
        return msg._connection_header["latching"] == "1"
