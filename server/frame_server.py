import pickle
import socketserver
from multiprocessing import Value
from multiprocessing.shared_memory import SharedMemory
import socket

import numpy as np

from utils import IMG_SIZE


class FrameHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        frame = pickle.loads(self.request[0].strip())
        addr = self.client_address[0]

        self.server.recv_frame(frame, addr)


class FrameServer:
    def __init__(self, loc: str, running: Value):
        mem = SharedMemory(name=loc)
        self.frame = np.ndarray(IMG_SIZE, dtype=np.uint8, buffer=mem.buf)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sf = self.socket.makefile(mode='rb')
        self.running = running
        self.last_active = {}

        adds = self.server.server_address
        print(f'[FS] Serving on {adds}')

    def __receive_frames(self, frame: np.ndarray, addr: str):
        np.copyto(self.frame, frame)
        if addr not in self.last_active:
            self.last_active[addr] = 0
            print(f'[FS] Receiving frames from {addr}')

    def run(self):
        self.socket.bind(('0.0.0.0', 9999))
        self.socket.listen(1)
        conn, addr = self.socket.accept()
        print(f'[FS] Connection from {addr}')
        with conn:
            frame = pickle.load(self.sf)
