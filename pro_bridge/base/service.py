import gzip
import itertools
import json
import threading

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Union, TYPE_CHECKING
from base.client import BridgeClientTCP

if TYPE_CHECKING:
    from base.ros1.node import ProBridgeRos1
    from base.ros2.node import ProBridgeRos2

# Message kinds ("k" header field). Topic messages have no "k".
KIND_ADVERTISE = "adv"  # the sender serves service "n" of type "t"
KIND_REQUEST = "req"
KIND_RESPONSE = "res"

DEFAULT_TIMEOUT = 5.0


def make_packet(version: int, srv_type: str, srv_name: str, kind: str, call_id: int, payload: bytes, compression_level: int = 0) -> bytes:
    json_compressed = gzip.compress(
        json.dumps({
            "v": version,
            "t": srv_type,
            "n": srv_name,
            "q": 0,
            "l": False,
            "c": compression_level,
            "k": kind,
            "id": call_id,
        }).encode("utf-8"),
        compresslevel=1,
    )
    if compression_level > 0:
        payload = gzip.compress(payload, compresslevel=compression_level)

    json_length = len(json_compressed).to_bytes(length=2, byteorder="little")
    return json_length + json_compressed + payload


def send_to_clients(tcp_clients: List[BridgeClientTCP], packet: bytes) -> bool:
    sent = False
    for client in tcp_clients:
        if client.is_connected() and client.send(packet):
            sent = True
    return sent


def mark_failed(response, reason: str):
    """Common convention (std_srvs/Trigger, SetBool and alike): success + message."""
    if hasattr(response, "success"):
        response.success = False
    if hasattr(response, "message"):
        response.message = reason
    return response


class PendingCall:
    def __init__(self) -> None:
        self.event = threading.Event()
        self.payload: Optional[bytes] = None


class ServiceCalls:
    """Calls to remote services waiting for a response, matched by id."""

    def __init__(self) -> None:
        self.__lock = threading.Lock()
        self.__ids = itertools.count(1)
        self.__pending: Dict[int, PendingCall] = {}

    def begin(self):
        call = PendingCall()
        with self.__lock:
            call_id = next(self.__ids)
            self.__pending[call_id] = call
        return call_id, call

    def end(self, call_id: int):
        with self.__lock:
            self.__pending.pop(call_id, None)

    def resolve(self, call_id: int, payload: bytes) -> bool:
        with self.__lock:
            call = self.__pending.get(call_id)
        if call is None:
            return False  # timed out or unknown
        call.payload = payload
        call.event.set()
        return True


class RemoteService(ABC):
    """
    ROS service served by the remote side (Unity). Created when the remote side advertises it;
    every call is forwarded to the remote hosts, the first response with the same id is returned.
    """

    def __init__(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"], tcp_clients: List[BridgeClientTCP],
                 srv_name: str, srv_type: str, timeout: float = DEFAULT_TIMEOUT) -> None:
        self.bridge = bridge
        self.tcp_clients = tcp_clients
        self.srv_name = srv_name
        self.srv_type = srv_type
        self.timeout = timeout
        self.create_srv()

    @abstractmethod
    def create_srv(self):
        """Create ROS service"""

    @abstractmethod
    def serialize_request(self, request) -> bytes:
        """Serialize ROS request"""

    @abstractmethod
    def deserialize_response(self, payload: bytes):
        """Deserialize ROS response"""

    @property
    @abstractmethod
    def ros_version(self) -> int:
        """ROS version written to the message header"""

    def call(self, request, response):
        """Forward the request and wait for the response. On failure returns `response` marked as failed."""
        call_id, call = self.bridge.service_calls.begin()
        try:
            packet = make_packet(self.ros_version, self.srv_type, self.srv_name, KIND_REQUEST, call_id,
                                 self.serialize_request(request))
            if not send_to_clients(self.tcp_clients, packet):
                return self.failed(response, "Remote side is not connected")

            if not call.event.wait(self.timeout):
                return self.failed(response, "No response within {:.1f} s".format(self.timeout))

            try:
                return self.deserialize_response(call.payload)
            except Exception as e:
                return self.failed(response, "Failed to deserialize response: {}".format(e))
        finally:
            self.bridge.service_calls.end(call_id)

    def failed(self, response, reason: str):
        self.bridge.logwarn("Service {}: {}".format(self.srv_name, reason))
        return mark_failed(response, reason)


class BridgeServiceClient(ABC):
    """
    Client of a ROS service listed in the config: the remote side (Unity) sends requests,
    responses go back to the hosts of the config group.
    """

    def __init__(self, bridge: Union["ProBridgeRos1", "ProBridgeRos2"], tcp_clients: List[BridgeClientTCP], settings: dict) -> None:
        self.bridge = bridge
        self.tcp_clients = tcp_clients
        self.srv_type = settings["type"]
        self.srv_name = settings["name"]
        self.timeout = float(settings.get("timeout", DEFAULT_TIMEOUT))
        self.compression_level = settings.get("compression_level", 0)
        self.create_client()

    @abstractmethod
    def create_client(self):
        """Create ROS service client"""

    @abstractmethod
    def handle_request(self, call_id: int, payload: bytes, reply_clients: Optional[List[BridgeClientTCP]] = None):
        """Call the ROS service; must end with send_response() (to reply_clients, default: the config group hosts)"""

    @property
    @abstractmethod
    def ros_version(self) -> int:
        """ROS version written to the message header"""

    def send_response(self, call_id: int, payload: bytes, reply_clients: Optional[List[BridgeClientTCP]] = None):
        packet = make_packet(self.ros_version, self.srv_type, self.srv_name, KIND_RESPONSE, call_id, payload,
                             self.compression_level)
        if not send_to_clients(reply_clients or self.tcp_clients, packet):
            self.bridge.logwarn("Service {}: response {} not sent, remote side is not connected".format(self.srv_name, call_id))
