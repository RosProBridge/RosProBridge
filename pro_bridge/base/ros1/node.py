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
        self._services_warned = False
        self.subscriber = None
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

    def create_bridge_service(self, base, clients, s):
        if not self._services_warned:
            self.logwarn("ROS service bridging is supported on ROS2 only; ignoring services in config")
            self._services_warned = True

    def destroy(self, *args):
        if getattr(self, "publisher", None) is not None:
            self.publisher.Stop()
        if getattr(self, "subscriber", None) is not None:
            self.subscriber.Stop()
        rospy.signal_shutdown(0)
