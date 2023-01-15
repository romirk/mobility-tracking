import pickle
import socket
import socketserver
from multiprocessing import Value
from multiprocessing.shared_memory import SharedMemory

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
    def __init__(self, running: Value):
        self.mem = SharedMemory(create=True, size=np.ndarray(IMG_SIZE, dtype=np.uint8).nbytes)
        self.frame = np.ndarray(IMG_SIZE, dtype=np.uint8, buffer=self.mem.buf)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.running = running
        self.last_active = {}

    def run(self):
        self.socket.bind(('0.0.0.0', 9999))
        self.socket.listen(1)
        print('[FS] Listening for frames')
        conn, addr = self.socket.accept()
        print(f'[FS] Connection from {addr}')
        with conn:
            sf = conn.makefile(mode='rb')
            while self.running.value:
                frame: np.ndarray = np.frombuffer(sf.read(self.frame.nbytes), dtype=np.uint8,
                                                  ).reshape(IMG_SIZE)
                # print(f'[FS] Received frame from {addr}: {frame.dtype}')
                np.copyto(self.frame, frame)
                # print(f'[FS] Copied frame to shared memory')
                if addr not in self.last_active:
                    self.last_active[addr] = 0
                    print(f'[FS] Receiving frames from {addr}')
            self.mem.close()
            self.mem.unlink()
