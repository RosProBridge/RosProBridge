import time
import socket

from urllib.parse import SplitResult
from typing import Optional
from threading import Thread


DELIMETER = b"\r\n"


class BridgeClientTCP:
    __connected = False
    __disposed = False
    __socket: Optional[socket.socket] = None

    def __init__(self, host: SplitResult, cb):
        self.__hostname = host.hostname
        self.__port = host.port
        self.cb = cb

        Thread(target=self.__check_connection, daemon=True).start()

    def Stop(self):
        try:
            if self.__socket != None:
                self.__socket.shutdown(0)
                self.__socket.close()
        except:
            pass
        self.__disposed = True
        self.__connected = False

    def __connect(self):
        if self.__socket is not None:
            try:
                self.__socket.shutdown(0)
            except:
                pass

        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

        try:
            self.__socket.connect((self.__hostname, self.__port))
        except Exception as e:
            return False

        return True

    def __check_connection(self):
        while not self.__disposed:
            if not self.__connected:
                self.__connected = self.__connect()
                if not self.__connected:
                    time.sleep(3)
                    continue
                else:
                    self.cb(self)
                    continue
            else:
                try:
                    self.__socket.send(DELIMETER)  # keep-alive #type: ignore
                    time.sleep(1)
                except:
                    self.__connected = False

    def is_connected(self) -> bool:
        return self.__connected

    def send(self, data):
        if self.__connected and self.__socket is not None:
            try:
                self.__socket.sendall(data + DELIMETER)
                return True
            except Exception as e:
                self.__connected = False
                return False
        else:
            return False
