#!/usr/bin/env python3

import rospy
import serial
from sensorbox.msg import AQI, AQIStamped


def aqi_node():
    _ = rospy.init_node("aqi")
    prefix = rospy.get_param("prefix")
    transport = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1.0)
    pub = rospy.Publisher(f"/{prefix}/aqdata", AQIStamped, queue_size=10)

    while not rospy.is_shutdown():
        if transport.in_waiting > 0:
            pub.publish(
                AQIStamped(
                    aqinfo=AQI(
                        *map(
                            float,
                            transport.readline().decode("utf-8").strip().split(","),
                        )
                    )
                )
            )


if __name__ == "__main__":
    aqi_node()
