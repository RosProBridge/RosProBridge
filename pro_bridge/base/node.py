import urllib.parse
import json

from abc import ABC, abstractmethod

class ProBridgeBase(ABC):
    def __init__(self, cfg: dict):
        # Parse host settings
        self.host = urllib.parse.urlsplit('//' + cfg['host']) #type: urllib.parse.SplitResult

        self.create_bridge_publisher()

        for p in cfg['published']:
            # Parse remote hosts
            clients = []
            for h in p['hosts']:
                clients.append(urllib.parse.urlsplit('//' + h))

            for t in p['topics']:
                self.create_bridge_subscriber(self, clients, t)

    @abstractmethod
    def destroy(self, *args):
        """Destroy all"""

    @abstractmethod
    def create_bridge_publisher(self):
        """Create BridgePublisher which will listen for UDP and TCP sockets"""

    @abstractmethod
    def create_bridge_subscriber(self, bridge, clients, t):
        """Create BridgeSubscriber which will listen for ROS messages and publish them to UDP | TCP"""

    @staticmethod
    def read_config(config_path: str) -> dict:
        with open(config_path, 'r') as json_file:
            cfg = json.load(json_file)
        return cfg