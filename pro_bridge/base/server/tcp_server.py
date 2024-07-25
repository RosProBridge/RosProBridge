import socket

from urllib.parse import SplitResult
from threading import Thread
from base.client.tcp_client import DELIMETER


class BridgeServerTCP:
    __disposed = False

    def __init__(self, host: SplitResult, cb):
        self.cb = cb
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.__socket.bind((host.hostname, host.port))
        self.__socket.listen(5)  # Allow the server to accept connections
        Thread(target=self.__accept_connections, daemon=True).start()

    def Stop(self):
        self.__disposed = True
        if self.__socket:
            try:
                self.__socket.shutdown(socket.SHUT_RDWR)
            except Exception as e:
                print(f"Error shutting down socket: {e}")
            self.__socket.close()

    def __accept_connections(self):
        while not self.__disposed:
            try:
                client_socket, addr = self.__socket.accept()
                Thread(target=self.__receive, args=(client_socket,)).start()
            except Exception as e:
                break

    def __receive(self, client_socket: socket.socket):
        buffer = bytes()
        while not self.__disposed:
            try:
                data = client_socket.recv(1024)
                if data:
                    buffer += data
                    while DELIMETER in buffer:
                        message, buffer = buffer.split(DELIMETER, 1)
                        if len(message) != 0 and self.cb:
                            self.cb(message)
                else:
                    break  # Connection closed
            except Exception as e:
                break
        client_socket.close()
