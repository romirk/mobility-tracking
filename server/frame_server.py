import socketserver
from multiprocessing import Value
from multiprocessing.shared_memory import SharedMemory

import numpy as np

from utils import IMG_SIZE, decode64


class FrameHandler(socketserver.StreamRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        data = self.rfile.readline().strip()
        frame: np.ndarray = decode64(data)
        addr = self.client_address[0]

        self.server.recv_frame(frame, addr)


class FrameServer:
    def __init__(self, loc: str, running: Value):
        mem = SharedMemory(name=loc)
        self.frame = np.ndarray(IMG_SIZE, dtype=np.uint8, buffer=mem.buf)
        self.server = socketserver.TCPServer(("localhost", 9999), FrameHandler)
        self.server.recv_frame = self.__receive_frames
        self.running = running
        self.last_active = {}

        adds = ', '.join(str(sock.getsockname()) for sock in self.server.sockets)
        print(f'[FS] Serving on {adds}')

    def __receive_frames(self, frame: np.ndarray, addr: str):
        np.copyto(self.frame, frame)
        if addr not in self.last_active:
            self.last_active[addr] = 0
            print(f'[FS] Receiving frames from {addr}')

    def run(self):
        self.server.serve_forever()
