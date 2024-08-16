import zmq

from urllib.parse import SplitResult
from threading import Thread


class BridgeServerTCP:
    __disposed = False

    def __init__(self, host: SplitResult, cb):
        self.cb = cb
        self.__context = zmq.Context()
        self.__socket = self.__context.socket(zmq.SUB)
        self.__socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.__socket.connect(f"tcp://{host.hostname}:{host.port}")
        Thread(target=self.__receive, daemon=True).start()

    def Stop(self):
        self.__disposed = True
        if self.__socket:
            self.__socket.close()

    def __receive(self):
       while not self.__disposed:
            message = self.__socket.recv()
            self.cb(message)