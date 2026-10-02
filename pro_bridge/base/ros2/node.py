import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from base.node import ProBridgeBase
from base.ros2.publisher import BridgePublisherRos2
from base.ros2.subscriber import BridgeSubscriberRos2
from base.ros2.service import RemoteServiceRos2, BridgeServiceClientRos2, failed_response_bytes
from base.service import make_packet, send_to_clients, KIND_RESPONSE


class ProBridgeRos2(ProBridgeBase, Node):
    def __init__(self, cfg: dict):
        Node.__init__(self, "ProBridge_" + cfg["id"])  # type: ignore
        self.loginfo = self.get_logger().info
        self.logwarn = self.get_logger().warning
        self.logerr = self.get_logger().error
        self.debug = self.get_logger().debug
        # Service calls wait for the remote response; they must not block topics or each other.
        self.service_callback_group = ReentrantCallbackGroup()
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

    def create_bridge_service_client(self, base, clients, s):
        return BridgeServiceClientRos2(base, clients, s)

    def create_remote_service(self, srv_name: str, srv_type: str, tcp_clients: list, timeout: float):
        return RemoteServiceRos2(self, tcp_clients, srv_name, srv_type, timeout)

    def send_service_failure(self, header: dict, reason: str, tcp_clients: list):
        name, srv_type = header.get("n", ""), header.get("t", "")
        self.logwarn("Service {}: {}".format(name, reason))
        try:
            payload = failed_response_bytes(srv_type, reason)
        except Exception as e:
            self.logerr("Service {}: can't build a response of type {}: {}".format(name, srv_type, e))
            return
        packet = make_packet(2, srv_type, name, KIND_RESPONSE, int(header.get("id", 0)), payload)
        send_to_clients(tcp_clients, packet)

    def destroy(self, *args):
        self.publisher.Stop()
        if hasattr(self, "subscriber"):  # a config may have services only
            self.subscriber.Stop()
        self.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

    def Spin(self):
        try:
            executor = MultiThreadedExecutor()
            executor.add_node(self)
            executor.spin()
        except:
            pass
        finally:
            self.destroy()
