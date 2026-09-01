import urllib.parse
import json

from abc import ABC, abstractmethod
from base.client import BridgeClientTCP


class ProBridgeBase(ABC):
    def __init__(self, cfg: dict):
        # Parse host settings
        self.host = urllib.parse.urlsplit('//' + cfg['host'])  # type: urllib.parse.SplitResult
        self.public_host = cfg.get('public_host')
        if not self.public_host and self.host.hostname not in (None, '0.0.0.0', '::', '[::]'):
            self.public_host = cfg['host']

        self.create_bridge_publisher()

        for p in cfg['published']:
            # Parse remote hosts
            clients = []
            for h in p['hosts']:
                clients.append(urllib.parse.urlsplit('//' + h))

            tcp_clients = []
            for client in clients:
                tcp_client = BridgeClientTCP(client)
                tcp_clients.append(tcp_client)

            for t in p.get('topics', []):
                self.create_bridge_subscriber(self, tcp_clients, t)

            for s in p.get('services', []):
                self.create_bridge_service(self, tcp_clients, s)

    @abstractmethod
    def destroy(self, *args):
        """Destroy all"""

    @abstractmethod
    def create_bridge_publisher(self):
        """Create BridgePublisher which will listen for UDP and TCP sockets"""

    @abstractmethod
    def create_bridge_subscriber(self, bridge, clients, t):
        """Create BridgeSubscriber which will listen for ROS messages and publish them to UDP | TCP"""

    @abstractmethod
    def create_bridge_service(self, bridge, clients, s):
        """Create bridged ROS service proxy (ROS2) or no-op (ROS1)"""

    @staticmethod
    def read_config(config_path: str) -> dict:
        with open(config_path, 'r') as json_file:
            cfg = json.load(json_file)
        return cfg
