import threading
import time

from pydoc import locate
from typing import TYPE_CHECKING, Dict, List

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.serialization import deserialize_message, serialize_message

from base.client import BridgeClientTCP
from base.service import (
    DEFAULT_SERVICE_TIMEOUT,
    KIND_SRV_REQ,
    KIND_SRV_RES,
    ServicePendingMap,
    new_call_id,
    pack_bridge_packet,
)

if TYPE_CHECKING:
    from base.ros2.node import ProBridgeRos2

class BridgeServiceProxyRos2:
    """Advertises a local ROS2 service and forwards calls over PUSH/PULL."""

    def __init__(
        self,
        bridge: "ProBridgeRos2",
        tcp_clients: List[BridgeClientTCP],
        settings: dict,
        public_host: str,
        pending: ServicePendingMap,
        callback_group: ReentrantCallbackGroup,
    ) -> None:
        self.bridge = bridge
        self.__tcp_clients = tcp_clients
        self.pending = pending
        self.public_host = public_host
        self.msg_name = settings["name"]
        self.msg_type = settings["type"]
        self.compression_level = settings.get("compression_level", 0)
        self.timeout = float(settings.get("timeout", DEFAULT_SERVICE_TIMEOUT))

        self.srv_cls = locate(self.msg_type)
        if self.srv_cls is None:
            raise ValueError("Unknown service type: {}".format(self.msg_type))

        self.__service = bridge.create_service(
            self.srv_cls,
            self.msg_name,
            self.__on_request,
            callback_group=callback_group,
        )
        bridge.loginfo('Created service proxy for "{}"'.format(self.msg_name))

    def __on_request(self, request, response):
        call_id = new_call_id()
        self.pending.register(call_id)

        try:
            payload = serialize_message(request)
        except Exception as e:
            self.bridge.logerr("Failed to serialize service request for {}: {}".format(self.msg_name, e))
            raise

        packet = pack_bridge_packet(
            {
                "v": 2,
                "t": self.msg_type,
                "n": self.msg_name,
                "c": self.compression_level,
                "k": KIND_SRV_REQ,
                "i": call_id,
                "r": self.public_host,
            },
            payload,
            self.compression_level,
        )

        if not self.__send(packet):
            self.pending.complete(call_id, error="failed to send service request")
            _, error = self.pending.wait(call_id, 0.0)
            raise RuntimeError(error or "Failed to send bridged service request for {}".format(self.msg_name))

        binary_response, error = self.pending.wait(call_id, self.timeout)
        if error:
            self.bridge.logerr("Service call {} failed: {}".format(self.msg_name, error))
            raise RuntimeError(error)
        if binary_response is None:
            raise RuntimeError("Empty service response for {}".format(self.msg_name))

        try:
            return deserialize_message(binary_response, self.srv_cls.Response)
        except Exception as e:
            self.bridge.logerr("Failed to deserialize service response for {}: {}".format(self.msg_name, e))
            raise

    def __send(self, msg: bytes) -> bool:
        sent = False
        try:
            for client in self.__tcp_clients:
                deadline = time.time() + 2.0
                while not client.is_connected():
                    if time.time() >= deadline:
                        break
                    time.sleep(0.01)
                if client.is_connected() and client.send(msg):
                    sent = True
        except Exception as e:
            self.bridge.logwarn("Can't transit service request: " + str(e))
        return sent

    def Stop(self):
        try:
            self.bridge.destroy_service(self.__service)
        except Exception:
            pass


class BridgeServiceProviderRos2:
    """Handles inbound bridged service requests by calling local ROS2 services."""

    def __init__(self, bridge: "ProBridgeRos2", callback_group: ReentrantCallbackGroup) -> None:
        self.bridge = bridge
        self._callback_group = callback_group
        self._clients: Dict[str, object] = {}
        self._lock = threading.Lock()

    def handle_request(self, json_data: dict, binary_packet: bytes) -> None:
        service_name = json_data.get("n", "")
        service_type = json_data.get("t", "")
        call_id = json_data.get("i")
        reply_to = json_data.get("r")
        compression_level = json_data.get("c", 0)

        if not call_id or not reply_to:
            self.bridge.logerr("Service request missing id or reply_to for {}".format(service_name))
            return

        srv_cls = locate(service_type)
        if srv_cls is None:
            self.__reply_error(json_data, "unknown service type: {}".format(service_type))
            return

        try:
            request = deserialize_message(binary_packet, srv_cls.Request)
        except Exception as e:
            self.__reply_error(json_data, "failed to deserialize request: {}".format(e))
            return

        client = self.__get_client(service_name, srv_cls)
        timeout = DEFAULT_SERVICE_TIMEOUT

        if not client.wait_for_service(timeout_sec=timeout):
            self.__reply_error(json_data, "service not available: {}".format(service_name))
            return

        future = client.call_async(request)
        done = threading.Event()

        def _done_cb(fut):
            done.set()

        future.add_done_callback(_done_cb)
        if not done.wait(timeout):
            self.__reply_error(json_data, "local service call timed out: {}".format(service_name))
            return

        try:
            result = future.result()
        except Exception as e:
            self.__reply_error(json_data, "local service call failed: {}".format(e))
            return

        try:
            payload = serialize_message(result)
        except Exception as e:
            self.__reply_error(json_data, "failed to serialize response: {}".format(e))
            return

        packet = pack_bridge_packet(
            {
                "v": 2,
                "t": service_type,
                "n": service_name,
                "c": compression_level,
                "k": KIND_SRV_RES,
                "i": call_id,
            },
            payload,
            compression_level,
        )
        if not self.bridge.client_registry.send(reply_to, packet):
            self.bridge.logerr("Failed to send service response for {} to {}".format(service_name, reply_to))

    def __get_client(self, service_name: str, srv_cls):
        with self._lock:
            client = self._clients.get(service_name)
            if client is None:
                client = self.bridge.create_client(srv_cls, service_name, callback_group=self._callback_group)
                self._clients[service_name] = client
            return client

    def __reply_error(self, json_data: dict, error: str) -> None:
        self.bridge.logerr(error)
        call_id = json_data.get("i")
        reply_to = json_data.get("r")
        if not call_id or not reply_to:
            return
        packet = pack_bridge_packet(
            {
                "v": 2,
                "t": json_data.get("t", ""),
                "n": json_data.get("n", ""),
                "c": 0,
                "k": KIND_SRV_RES,
                "i": call_id,
                "e": error,
            },
            b"",
            0,
        )
        self.bridge.client_registry.send(reply_to, packet)

    def Stop(self):
        with self._lock:
            for client in self._clients.values():
                try:
                    self.bridge.destroy_client(client)
                except Exception:
                    pass
            self._clients.clear()
