import gzip

from abc import ABC, abstractmethod
from typing import Union, Optional, List, Dict, TYPE_CHECKING
from base.client import BridgeClientTCP

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1
    from base.ros2.node import ProBridgeRos2

MAX_UDP_SIZE = 1450


class BridgeSubscriber(ABC):
    msg_qos: Optional[Union[str, int, Dict[str,str]]]
    msg = None

    def __init__(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"], tcp_clients: List[BridgeClientTCP], settings: dict) -> None:
        super().__init__()
        self.bridge = bridge
        self.__tcp_clients = tcp_clients

        for tcp_client in self.__tcp_clients:
            tcp_client.cb = self.on_tcp_connection

        self.msg_type = settings["type"]
        self.msg_name = settings["name"]
        self.compression_level = settings.get("compression_level", 0)

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
    def prepare_msg(self, m_type: str, m_name: str, m_qos, m_data) -> bytes:
        """Prepare message structure"""

    @abstractmethod
    def getrostime(self) -> float:
        """Get ROS Time"""

    @abstractmethod
    def is_latch_msg(self, msg) -> bool:
        """Check if the message is latched. Make sure latch parameter will be overriden with json settings"""

    def msg_callback(self, msg):
        packet = self.prepare_msg(self.msg_type, self.msg_name, self.msg_qos, msg)

        if self.is_latch_msg(msg):
            self.msg = packet  # Store messages received through the TCP socket in a variable, so they can be sent to newly connected TCP clients

        current_time = self.getrostime()
        if current_time - self.__msg_send_time >= self.__msg_rate:
            self.send_msg(packet)
            self.__msg_send_time = current_time

    def on_tcp_connection(self, tcp_client: BridgeClientTCP):
        """Send last latch message on TCP connection"""
        if self.msg is not None:
            tcp_client.send(self.msg)

    def send_msg(self, msg: bytes):
        try:
            for client in self.__tcp_clients:
                if client.is_connected():
                    client.send(msg)
        except Exception as e:
            self.bridge.logwarn("Can't transit message " + str(e))
            pass

    def Stop(self):
        for client in self.__tcp_clients:
            client.Stop()
