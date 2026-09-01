import time
import urllib.parse

from threading import Lock
from typing import Dict

from base.client.tcp_client import BridgeClientTCP


class BridgeClientRegistry:
    """Cache of outbound PUSH clients keyed by host:port (for service reply_to)."""

    def __init__(self) -> None:
        self._clients: Dict[str, BridgeClientTCP] = {}
        self._lock = Lock()

    def get(self, host: str) -> BridgeClientTCP:
        with self._lock:
            client = self._clients.get(host)
            if client is None:
                parsed = urllib.parse.urlsplit("//" + host)
                client = BridgeClientTCP(parsed)
                self._clients[host] = client
            return client

    def send(self, host: str, data: bytes, connect_timeout: float = 2.0) -> bool:
        client = self.get(host)
        deadline = time.time() + connect_timeout
        while not client.is_connected():
            if time.time() >= deadline:
                return False
            time.sleep(0.01)
        return client.send(data)

    def Stop(self):
        with self._lock:
            for client in self._clients.values():
                client.Stop()
            self._clients.clear()
