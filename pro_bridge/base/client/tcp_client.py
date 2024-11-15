import zmq

from zmq.utils.monitor import recv_monitor_message
from threading import Thread
from urllib.parse import SplitResult

ZMQ_INT_VERSION = int(zmq.__version__.split('.')[0])
if ZMQ_INT_VERSION <= 22:
    ZMQ_EVENT_HANDSHAKE_SUCCEEDED = zmq.EVENT_HANDSHAKE_SUCCEEDED
    ZMQ_EVENT_DISCONNECTED = zmq.EVENT_DISCONNECTED
else:
    ZMQ_EVENT_HANDSHAKE_SUCCEEDED = zmq.Event.HANDSHAKE_SUCCEEDED
    ZMQ_EVENT_DISCONNECTED = zmq.Event.DISCONNECTED

class BridgeClientTCP:
    __connected = False

    def __init__(self, host: SplitResult, cb: list = []):
        self.__hostname = host.hostname
        self.__port = host.port
        self.cb = []

        self.__context = zmq.Context()
        self.__socket = self.__context.socket(zmq.PUSH)
        self.__socket.connect(f"tcp://{self.__hostname}:{self.__port}")
        self.__monitor = self.__socket.get_monitor_socket()
        Thread(target=self.__monitor_socket, daemon=True).start()

    def __monitor_socket(self):
        while self.__monitor.poll():
            event = recv_monitor_message(self.__monitor)

            value = event.get("event")
            if value == ZMQ_EVENT_HANDSHAKE_SUCCEEDED:
                self.__connected = True
                for cb in self.cb:
                    cb(self)
            elif value == ZMQ_EVENT_DISCONNECTED:
                self.__connected = False

    def Stop(self):
        try:
            if self.__socket != None:
                self.__socket.close()
        except:
            pass
        self.__connected = False

    def is_connected(self) -> bool:
        return self.__connected

    def send(self, data):
        if self.__connected:
            try:
                self.__socket.send(data)
                return True
            except Exception as e:
                self.__connected = False
                return False
        else:
            return False
