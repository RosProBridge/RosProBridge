import urllib.parse
import json

from typing import Optional

from abc import ABC, abstractmethod
from base.client import BridgeClientTCP
from base.service import ServiceCalls, DEFAULT_TIMEOUT

class ProBridgeBase(ABC):
    def __init__(self, cfg: dict):
        # Parse host settings
        self.host = urllib.parse.urlsplit('//' + cfg['host']) #type: urllib.parse.SplitResult
        self.service_calls = ServiceCalls()
        self.remote_services = {}   # served by the remote side, created on its advertisement
        self.service_clients = {}   # ROS services from the config, called by the remote side
        # All remote hosts: from the config and from service advertisements
        self.remote_clients = []

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
                self.remote_clients.append(tcp_client)

            for t in p.get('topics', []):
                self.create_bridge_subscriber(self, tcp_clients, t)

            for s in p.get('services', []):
                try:
                    self.service_clients[s["name"]] = self.create_bridge_service_client(self, tcp_clients, s)
                except Exception as e:
                    self.logerr("Failed to create service client {}: {}".format(s.get("name", ""), e))

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
    def create_bridge_service_client(self, bridge, clients, s):
        """Create BridgeServiceClient: a ROS service from the config called by the remote side"""

    @abstractmethod
    def create_remote_service(self, srv_name: str, srv_type: str, tcp_clients: list, timeout: float):
        """Create RemoteService: a ROS service served by the remote side"""

    @abstractmethod
    def send_service_failure(self, header: dict, reason: str, tcp_clients: list):
        """Answer a request for an unknown service with a failed response"""

    # Called from the receiving thread

    def get_remote_client(self, hostname: str, port: int) -> BridgeClientTCP:
        for client in self.remote_clients:
            if client.hostname == hostname and client.port == port:
                return client
        client = BridgeClientTCP(urllib.parse.urlsplit("//{}:{}".format(hostname, port)))
        self.remote_clients.append(client)
        self.loginfo("Connect to remote host {}:{}".format(hostname, port))
        return client

    def get_sender_client(self, header: dict, peer: Optional[str]) -> Optional[BridgeClientTCP]:
        """Client to the sender of a service message: its IP from the connection, its server port from the header"""
        port = header.get("p")
        if not peer or not port:
            return None
        return self.get_remote_client(peer, int(port))

    def on_service_advertise(self, header: dict, peer: Optional[str] = None):
        name, srv_type = header.get("n", ""), header.get("t", "")

        # Calls go to the advertiser; without its address (older remote side) - to all known hosts.
        sender = self.get_sender_client(header, peer)
        clients = [sender] if sender else self.remote_clients

        if name in self.remote_services:
            service = self.remote_services[name]
            if service is None:
                return
            if service.srv_type != srv_type:
                self.logerr("Service {} is already advertised as {}, {} ignored".format(name, service.srv_type, srv_type))
                return
            for client in clients:
                if client not in service.tcp_clients:
                    service.tcp_clients.append(client)
            return

        try:
            self.remote_services[name] = self.create_remote_service(name, srv_type, clients, DEFAULT_TIMEOUT)
        except Exception as e:
            self.logerr("Failed to create service {}: {}".format(name, e))
            self.remote_services[name] = None  # don't retry on every advertisement

    def on_service_request(self, header: dict, payload: bytes, peer: Optional[str] = None):
        # The response goes to the caller; without its address - to the hosts of the config group.
        sender = self.get_sender_client(header, peer)
        reply_clients = [sender] if sender else None

        client = self.service_clients.get(header.get("n", ""))
        if client is None:
            self.send_service_failure(header, "Service is not listed in the bridge config", reply_clients or self.remote_clients)
            return
        client.handle_request(int(header.get("id", 0)), payload, reply_clients)

    def on_service_response(self, header: dict, payload: bytes):
        if not self.service_calls.resolve(int(header.get("id", 0)), payload):
            self.logwarn("Late or unknown response for service {} (id {}) ignored".format(header.get("n", ""), header.get("id")))

    @staticmethod
    def read_config(config_path: str) -> dict:
        with open(config_path, 'r') as json_file:
            cfg = json.load(json_file)
        return cfg