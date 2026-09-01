import gzip
import json
import threading
import uuid

from typing import Dict, Optional, Tuple


KIND_SRV_REQ = 1
KIND_SRV_RES = 2
DEFAULT_SERVICE_TIMEOUT = 5.0


class PendingCall:
    def __init__(self) -> None:
        self.event = threading.Event()
        self.payload: Optional[bytes] = None
        self.error: Optional[str] = None


class ServicePendingMap:
    def __init__(self) -> None:
        self._pending: Dict[str, PendingCall] = {}
        self._lock = threading.Lock()

    def register(self, call_id: str) -> PendingCall:
        pending = PendingCall()
        with self._lock:
            self._pending[call_id] = pending
        return pending

    def complete(self, call_id: str, payload: Optional[bytes] = None, error: Optional[str] = None) -> None:
        with self._lock:
            pending = self._pending.get(call_id)
        if pending is None:
            return
        pending.payload = payload
        pending.error = error
        pending.event.set()

    def wait(self, call_id: str, timeout: float) -> Tuple[Optional[bytes], Optional[str]]:
        with self._lock:
            pending = self._pending.get(call_id)
        if pending is None:
            return None, "unknown call id"
        finished = pending.event.wait(timeout)
        with self._lock:
            self._pending.pop(call_id, None)
        if not finished:
            return None, "timeout"
        return pending.payload, pending.error


def new_call_id() -> str:
    return str(uuid.uuid4())


def pack_bridge_packet(json_data: dict, payload: bytes, compression_level: int = 0) -> bytes:
    json_compressed = gzip.compress(json.dumps(json_data).encode("utf-8"), compresslevel=1)
    if compression_level > 0:
        serialized = gzip.compress(payload, compresslevel=compression_level)
    else:
        serialized = payload
    json_length = len(json_compressed).to_bytes(length=2, byteorder="little")
    return json_length + json_compressed + serialized
