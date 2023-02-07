#!/usr/bin/env python3

import rospy
import serial
from sensorbox.msg import AQI


def aqi_node():
    _ = rospy.init_node('aqi')
    transport = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1.0)
    pub = rospy.Publisher('/sbx/aqi', AQI, queue_size=10)

    while True:
        if transport.in_waiting > 0:
            pub.publish(AQI(*map(float, transport.readline().decode("utf-8").strip().split(','))))


if __name__ == '__main__':
    aqi_node()
