import rclpy

from rclpy.node import Node
from base.node import ProBridgeBase
from base.ros2.publisher import BridgePublisherRos2
from base.ros2.subscriber import BridgeSubscriberRos2


class ProBridgeRos2(ProBridgeBase, Node):
    def __init__(self, cfg: dict):
        Node.__init__(self, "ProBridge_" + cfg["id"])  # type: ignore
        self.loginfo = self.get_logger().info
        self.logwarn = self.get_logger().warning
        self.logerr = self.get_logger().error
        self.debug = self.get_logger().debug
        super().__init__(cfg)
        self.get_logger().info("ProBridge launched")
        self.Spin()

    def log(self, level: str, text: str):
        self.get_logger()

    @classmethod
    def start(cls, config_path: str):
        cfg = ProBridgeBase.read_config(config_path)
        rclpy.init()
        instance = cls(cfg)
        return instance

    def create_bridge_publisher(self):
        self.publisher = BridgePublisherRos2(self)

    def create_bridge_subscriber(self, base, clients, t):
        self.subscriber = BridgeSubscriberRos2(base, clients, t)

    def destroy(self, *args):
        self.publisher.Stop()
        self.subscriber.Stop()
        self.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

    def Spin(self):
        try:
            rclpy.spin(self)
        except:
            pass
        finally:
            self.destroy()
