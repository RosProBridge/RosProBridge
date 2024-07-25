import socket

from urllib.parse import SplitResult
from threading import Thread

class BridgeServerUDP:
    __is_active = True
    def __init__(self, host: SplitResult, cb):
        self.__hostname = host.hostname
        self.__port = host.port
        self.cb = cb

        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.__socket.bind((self.__hostname, self.__port))
        Thread(target=self.__recv_msg, args=(), daemon=True).start()

    def __recv_msg(self):
        while self.__is_active:
            try:
                bytesAddressPair = self.__socket.recvfrom(8192)
            except:
                continue

            if self.__is_active is False:
                return

            clientIP = bytesAddressPair[1][0]
            # skip loop messages
            if clientIP == self.__hostname:
                continue

            self.cb(bytesAddressPair[0])

    def Stop(self):
        self.__is_active = False
        try:
            self.__socket.shutdown(0)
        except:
            pass