#!../env/bin/python
from multiprocessing import Process, Value
from time import sleep

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
        self.server: str = config.server
        self.config = config

        print("[SensorBox] Starting...")

        self.running = Value('b', True)

    def serial_listener(self, running: Value):
        print(f"[SensorBox] Starting serial listener on port {self.port}...")
        self.serial = serial.Serial(self.port, self.baud)
        print(self.serial.is_open)
        while running.value:
            if self.serial.in_waiting > 0:
                pm10, pm25, pm50, pm100, tmp, hmd, co2 = tuple(map(float, self.serial.readline().decode(
                    "utf-8").strip().split(',')))
                print(pm10, pm25, pm50, pm100, tmp, hmd, co2)
                requests.post(f"{self.server}/sensors",
                              json={"pm10": pm10, "pm25": pm25, "pm50": pm50, "pm100": pm100, "tmp": tmp, "hmd": hmd,
                                    "co2": co2})
            sleep(1.5)
        print("[SensorBox] Stopping serial listener...")

    def count_traffic(self, running: Value):
        self.traffic_counter = TrafficCounter(self.config)
        self.traffic_counter.main_loop(running)

    def run(self):
        try:
            if self.config.parallel:
                print("starting threads")
                count_process = Process(target=self.count_traffic, args=(self.running,))
                sensor_process = Process(target=self.serial_listener, args=(self.running,))
                sensor_process.start()
                count_process.start()

                sensor_process.join()
                count_process.join()
            else:

                self.count_traffic(self.running)
        except KeyboardInterrupt:
            print("\n[SensorBox] Stopping...")
            self.running.value = False
        finally:
            print("[SensorBox] terminated.")


if __name__ == "__main__":
    opts = parse_args()
    print(opts)
    box = SensorBox(opts)
    box.run()
