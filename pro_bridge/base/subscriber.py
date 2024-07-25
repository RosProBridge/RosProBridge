import gzip

from urllib.parse import SplitResult
from abc import ABC, abstractmethod
from typing import Union, Optional, List, TYPE_CHECKING
from base.client import BridgeClientTCP, BridgeClientUDP

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1
    from base.ros2.node import ProBridgeRos2

MAX_UDP_SIZE = 1450


class BridgeSubscriber(ABC):
    msg_qos = None
    msg = None

    def __init__(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"], clients: List[SplitResult], settings: dict) -> None:
        super().__init__()
        self.bridge = bridge
        self.__udp_client = BridgeClientUDP()
        self.__tcp_clients = []  # type: List[BridgeClientTCP]
        for h in clients:
            self.__tcp_clients.append(BridgeClientTCP(h, self.on_tcp_connection))

        self.__udp_hosts = clients

        self.msg_type = settings["type"]
        self.msg_name = settings["name"]

        rate = settings.get("rate", 0)
        self.__msg_rate = 0 if rate <= 0 else 1.0 / rate
        self.__msg_send_time = 0
        self.__is_latch = settings.get("latch")

        self.create_sub(bridge, settings)

    @property
    def is_latch(self) -> Optional[bool]:
        return self.__is_latch

    @abstractmethod
    def create_sub(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"], settings: dict):
        """Subscribe to ROS messages"""

    @abstractmethod
    def convert_msg(self, m_data):
        """Convert ROS message to dictionary"""

    @abstractmethod
    def prepare_msg(self, m_type: str, m_name: str, m_qos, m_data) -> str:
        """Convert ROS message to dictionary then pack to JSON"""

    @abstractmethod
    def getrostime(self) -> float:
        """Get ROS Time"""

    @abstractmethod
    def is_latch_msg(self, msg) -> bool:
        """Check if the message is latched. Make sure latch parameter will be overriden with json settings"""

    def msg_callback(self, msg):
        json_message = self.prepare_msg(self.msg_type, self.msg_name, self.msg_qos, msg)
        compressed_message = gzip.compress(bytes(json_message, "ascii"))

        if self.is_latch_msg(msg) or len(compressed_message) > MAX_UDP_SIZE:
            self.msg = compressed_message  # latch messages sends through TCP socket so save it to self variable for sending to new connected tcp clients
            is_tcp = True
        else:
            is_tcp = False

        current_time = self.getrostime()
        if is_tcp or current_time - self.__msg_send_time >= self.__msg_rate:
            self.send_msg(compressed_message, is_tcp)
            self.__msg_send_time = current_time

    def on_tcp_connection(self, tcp_client: BridgeClientTCP):
        """Send last latch message on TCP connection"""
        if self.msg is not None:
            tcp_client.send(self.msg)

    def send_msg(self, msg: bytes, is_tcp: bool):
        try:
            if is_tcp:
                for client in self.__tcp_clients:
                    if client.is_connected():
                        client.send(msg)
            else:
                for host in self.__udp_hosts:
                    self.__udp_client.send(msg, (host.hostname, host.port))
        except Exception as e:
            self.bridge.logwarn("Can't transit message " + str(e))
            pass

    def Stop(self):
        for client in self.__tcp_clients:
            client.Stop()
