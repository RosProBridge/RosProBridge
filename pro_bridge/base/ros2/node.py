import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from base.node import ProBridgeBase
from base.ros2.publisher import BridgePublisherRos2
from base.ros2.subscriber import BridgeSubscriberRos2
from base.ros2.service import BridgeServiceProxyRos2, BridgeServiceProviderRos2
from base.service import ServicePendingMap
from base.client.client_registry import BridgeClientRegistry


class ProBridgeRos2(ProBridgeBase, Node):
    def __init__(self, cfg: dict):
        Node.__init__(self, "ProBridge_" + cfg["id"])  # type: ignore
        self.loginfo = self.get_logger().info
        self.logwarn = self.get_logger().warning
        self.logerr = self.get_logger().error
        self.debug = self.get_logger().debug

        self.service_callback_group = ReentrantCallbackGroup()
        self.service_pending = ServicePendingMap()
        self.client_registry = BridgeClientRegistry()
        self.service_proxies = []
        self.service_provider = BridgeServiceProviderRos2(self, self.service_callback_group)
        self.subscriber = None

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

    def create_bridge_service(self, base, clients, s):
        if not self.public_host:
            self.logerr(
                "Cannot create service proxy for {}: public_host is required when host binds to 0.0.0.0".format(
                    s.get("name", "")
                )
            )
            return
        if not clients:
            self.logerr(
                "Cannot create service proxy for {}: no remote hosts configured".format(s.get("name", ""))
            )
            return
        try:
            proxy = BridgeServiceProxyRos2(
                bridge=self,
                tcp_clients=clients,
                settings=s,
                public_host=self.public_host,
                pending=self.service_pending,
                callback_group=self.service_callback_group,
            )
            self.service_proxies.append(proxy)
        except Exception as e:
            self.logerr("Failed to create service proxy for {}: {}".format(s.get("name", ""), e))

    def handle_service_request(self, json_data: dict, binary_packet: bytes):
        self.service_provider.handle_request(json_data, binary_packet)

    def handle_service_response(self, json_data: dict, binary_packet: bytes):
        call_id = json_data.get("i")
        if not call_id:
            self.logwarn("Service response missing correlation id")
            return
        error = json_data.get("e")
        if error:
            self.service_pending.complete(call_id, error=error)
        else:
            self.service_pending.complete(call_id, payload=binary_packet)

    def destroy(self, *args):
        if getattr(self, "publisher", None) is not None:
            self.publisher.Stop()
        if getattr(self, "subscriber", None) is not None:
            self.subscriber.Stop()
        for proxy in getattr(self, "service_proxies", []):
            proxy.Stop()
        if getattr(self, "service_provider", None) is not None:
            self.service_provider.Stop()
        if getattr(self, "client_registry", None) is not None:
            self.client_registry.Stop()
        self.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def Spin(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin()
        except Exception:
            pass
        finally:
            self.destroy()
