import socket
from argparse import Namespace

import serial


class SensorBox:
    serial: serial.Serial

    def __init__(self, config):
        self.port: str = config.port
        self.baud: int = config.baud
        self.server: tuple[str, int] = config.server
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.socket.connect(self.server)

    def serial_listener(self):
        self.serial = serial.Serial(self.port, self.baud)
        while True:
            if self.serial.in_waiting > 0:
                readouts = tuple(map(float, self.serial.readline().decode(
                    "utf-8").strip().split(',')))
                self.socket.sendall(bytes(str(readouts) + "\n", "utf-8"))


if __name__ == "__main__":
    opts = Namespace()
    opts.port = "COM4"
    opts.baud = 115200
    opts.server = ("localhost", 9999)
    box = SensorBox(opts)
    box.serial_listener()
