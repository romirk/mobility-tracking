#!/usr/bin/python3
import serial

transport = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1.0)
while True:
    if transport.in_waiting > 0:
        pm10, pm25, pm50, pm100, tmp, hmd, co2 = tuple(map(float, transport.readline().decode(
            "utf-8").strip().split(',')))
        print(pm10, pm25, pm50, pm100, tmp, hmd, co2)
