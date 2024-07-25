import rospy

from base.node import ProBridgeBase
from base.ros1.publisher import BridgePublisherRos1
from base.ros1.subscriber import BridgeSubscriberRos1


class ProBridgeRos1(ProBridgeBase):
    def __init__(self, cfg: dict):
        self.loginfo = rospy.loginfo
        self.logwarn = rospy.logwarn
        self.logerr = rospy.logerr
        self.logdebug = rospy.logdebug
        super().__init__(cfg)
        rospy.on_shutdown(self.destroy)
        rospy.spin()

    @classmethod
    def start(cls, config_path: str):
        cfg = ProBridgeBase.read_config(config_path)
        rospy.init_node("ProBridgeRos1_" + cfg["id"])
        rospy.loginfo("ProBridge launched")
        return cls(cfg)

    def create_bridge_publisher(self):
        self.publisher = BridgePublisherRos1(self)

    def create_bridge_subscriber(self, base, clients, t):
        self.subscriber = BridgeSubscriberRos1(base, clients, t)

    def destroy(self, *args):
        self.publisher.Stop()
        self.subscriber.Stop()
        rospy.signal_shutdown(0)
