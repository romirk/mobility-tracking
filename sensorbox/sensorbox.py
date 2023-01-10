from multiprocessing import Process, Value

import requests
import serial

from traffic_counter import TrafficCounter
from utils import parse_args


class SensorBox:
    serial: serial.Serial
    traffic_counter: TrafficCounter

    def __init__(self, config):
        self.port: str = config.port
        self.baud: int = config.baud
        self.server: tuple[str, int] = config.server
        self.config = config

        print("[SensorBox] Starting...")

        self.running = Value('b', True)

        self.count_process = Process(target=self.count_traffic, args=(self.running,))
        self.sensor_process = Process(target=self.serial_listener, args=(self.running,))

    def serial_listener(self, running: Value):
        print(f"[SensorBox] Starting serial listener on port {self.port}...")
        self.serial = serial.Serial(self.port, self.baud)
        while running.value:
            print("[SensorBox] Waiting for data...")
            if self.serial.in_waiting > 0:
                pm10, pm25, pm50, pm100, tmp, hmd, co2 = tuple(map(float, self.serial.readline().decode(
                    "utf-8").strip().split(',')))
                print(pm10, pm25, pm50, pm100, tmp, hmd, co2)
                requests.post(f"http://{self.server[0]}:{self.server[1]}/sensors",
                              json={"pm10": pm10, "pm25": pm25, "pm50": pm50, "pm100": pm100, "tmp": tmp, "hmd": hmd,
                                    "co2": co2})
        print("[SensorBox] Stopping serial listener...")

    def count_traffic(self, running: Value):
        self.traffic_counter = TrafficCounter(self.config)
        self.traffic_counter.main_loop(running)

    def run(self):
        try:
            self.sensor_process.start()
            self.count_process.start()

            self.sensor_process.join()
            self.count_process.join()
        except KeyboardInterrupt:
            print("[SensorBox] Stopping...")
            self.running.value = False


if __name__ == "__main__":
    opts = parse_args()
    box = SensorBox(opts)
    box.run()
