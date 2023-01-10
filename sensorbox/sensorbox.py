from argparse import Namespace

import requests
import serial


class SensorBox:
    serial: serial.Serial

    def __init__(self, config):
        self.port: str = config.port
        self.baud: int = config.baud
        self.server: tuple[str, int] = config.server

    def serial_listener(self):
        self.serial = serial.Serial(self.port, self.baud)
        while True:
            if self.serial.in_waiting > 0:
                pm10, pm25, pm50, pm100, tmp, hmd, co2 = tuple(map(float, self.serial.readline().decode(
                    "utf-8").strip().split(',')))
                print(pm10, pm25, pm50, pm100, tmp, hmd, co2)
                requests.post(f"http://{self.server[0]}:{self.server[1]}/sensors",
                              json={"pm10": pm10, "pm25": pm25, "pm50": pm50, "pm100": pm100, "tmp": tmp, "hmd": hmd,
                                    "co2": co2})


if __name__ == "__main__":
    opts = Namespace()
    opts.port = "COM4"
    opts.baud = 115200
    opts.server = ("localhost", 5000)
    box = SensorBox(opts)
    box.serial_listener()
