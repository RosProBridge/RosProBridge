import threading

from pydoc import locate
from rclpy.serialization import serialize_message, deserialize_message
from base.service import RemoteService, BridgeServiceClient, mark_failed
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from base.ros2.node import ProBridgeRos2


def locate_srv(srv_type: str):
    srv_class = locate(srv_type)
    if srv_class is None:
        raise ValueError("Unknown service type {}. Make sure the workspace with it is sourced.".format(srv_type))
    return srv_class


def failed_response_bytes(srv_type: str, reason: str) -> bytes:
    return serialize_message(mark_failed(locate_srv(srv_type).Response(), reason))


class RemoteServiceRos2(RemoteService):
    bridge: "ProBridgeRos2"

    def create_srv(self):
        self.srv_class = locate_srv(self.srv_type)
        # Calls block until the response arrives, so they run in their own reentrant group.
        self.bridge.create_service(self.srv_class, self.srv_name, self.call,
                                   callback_group=self.bridge.service_callback_group)
        self.bridge.loginfo('Advertised service "{}" ({})'.format(self.srv_name, self.srv_type))

    def serialize_request(self, request) -> bytes:
        return serialize_message(request)

    def deserialize_response(self, payload: bytes):
        return deserialize_message(payload, self.srv_class.Response)

    @property
    def ros_version(self) -> int:
        return 2


class BridgeServiceClientRos2(BridgeServiceClient):
    bridge: "ProBridgeRos2"

    def create_client(self):
        self.srv_class = locate_srv(self.srv_type)
        self.client = self.bridge.create_client(self.srv_class, self.srv_name,
                                                callback_group=self.bridge.service_callback_group)
        self.bridge.loginfo('Create service client "{}" ({})'.format(self.srv_name, self.srv_type))

    def handle_request(self, call_id: int, payload: bytes, reply_clients=None):
        try:
            request = deserialize_message(payload, self.srv_class.Request)
        except Exception as e:
            self.send_failure(call_id, "Failed to deserialize request: {}".format(e), reply_clients)
            return

        if not self.client.service_is_ready():
            self.send_failure(call_id, "Service is not available", reply_clients)
            return

        lock = threading.Lock()
        state = {"done": False}

        def finish(payload_or_reason: bytes = None, reason: str = None):
            with lock:
                if state["done"]:
                    return
                state["done"] = True
            timer.cancel()
            if reason is not None:
                self.send_failure(call_id, reason, reply_clients)
            else:
                self.send_response(call_id, payload_or_reason, reply_clients)

        def on_done(future):
            try:
                finish(serialize_message(future.result()))
            except Exception as e:
                finish(reason="Service call failed: {}".format(e))

        def on_timeout():
            future.cancel()
            finish(reason="No response within {:.1f} s".format(self.timeout))

        timer = threading.Timer(self.timeout, on_timeout)
        timer.daemon = True
        future = self.client.call_async(request)
        timer.start()
        future.add_done_callback(on_done)

    def send_failure(self, call_id: int, reason: str, reply_clients=None):
        self.bridge.logwarn("Service {}: {}".format(self.srv_name, reason))
        self.send_response(call_id, failed_response_bytes(self.srv_type, reason), reply_clients)

    @property
    def ros_version(self) -> int:
        return 2
