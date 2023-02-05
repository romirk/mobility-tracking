#!/usr/bin/python3

import rospy
import serial
from sensorbox.msg import AQI


def reader():
    node = rospy.init_node('aqi')
    transport = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1.0)
    pub = rospy.Publisher('/sbx/aqi', AQI, queue_size=10)
    while True:
        if transport.in_waiting > 0:
            pm10, pm25, pm50, pm100, tmp, hmd, co2 = tuple(map(float, transport.readline().decode(
                "utf-8").strip().split(',')))
            pub.publish(AQI(pm10, pm25, pm50, pm100, tmp, hmd, co2))


if __name__ == '__main__':
    reader()
