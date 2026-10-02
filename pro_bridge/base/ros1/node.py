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

    def create_bridge_service_client(self, base, clients, s):
        raise NotImplementedError("Services are not supported in ROS1 yet")

    def create_remote_service(self, srv_name: str, srv_type: str, tcp_clients: list, timeout: float):
        raise NotImplementedError("Services are not supported in ROS1 yet")

    def send_service_failure(self, header: dict, reason: str, tcp_clients: list):
        self.logwarn("Service {}: {} (services are not supported in ROS1 yet)".format(header.get("n", ""), reason))

    def destroy(self, *args):
        self.publisher.Stop()
        if hasattr(self, "subscriber"):  # a config may have services only
            self.subscriber.Stop()
        rospy.signal_shutdown(0)
