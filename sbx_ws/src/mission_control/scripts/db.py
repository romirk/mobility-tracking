#!/usr/bin/env python3

import pymongo
import rospy
from mission_control.msg import Counts, CountStamped
from mission_control.srv import Timeline, TimelineResponse
from sensorbox.msg import AQI, AQIStamped
from vision_msgs.msg import Detection2D


def record_to_count_stamped(record):
    return CountStamped(
        rospy.Time(record["stamp"].timestamp()),
        Counts(
            total=record["total"],
            cars=record["counts"]["cars"],
            trucks=record["counts"]["trucks"],
            buses=record["counts"]["buses"],
            motorcycles=record["counts"]["motorcycles"],
        ),
    )


def record_to_sensor_data(record):
    return AQIStamped(
        header=rospy.Header(stamp=rospy.Time(record["stamp"].timestamp())),
        aqinfo=AQI(
            pm25=record["pm25"],
            pm10=record["pm10"],
            pm50=record["pm50"],
            pm100=record["pm100"],
            co2=record["co2"],
            hum=record["hum"],
            tmp=record["tmp"],
        ),
    )


class MissionArchives:
    __CONN_STR = "mongodb://localhost:27017"

    def __init__(self) -> None:
        rospy.init_node("time-machine")
        prefix = rospy.get_param("prefix", "sbx")

        self.client = pymongo.MongoClient(MissionArchives.__CONN_STR)
        self.db = self.client.impactlab
        self.traffic_collection = self.db.mobility_tracking
        self.sensor_collection = self.db.sensor_data

        # services
        self.timeline_srv = rospy.Service(
            f"/{prefix}/timetravel/history", Timeline, self.timeline_callback
        )
        self.last_count_srv = rospy.Service(
            f"/{prefix}/timetravel/last", Timeline, self.last_count_callback
        )
        self.sensor_data_srv = rospy.Service(
            f"/{prefix}/timetravel/sensor", Timeline, self.sensor_data_callback
        )

        # subscribers
        self.traffic_sub = rospy.Subscriber(
            f"/{prefix}/detect", Detection2D, self.traffic_callback
        )
        self.sensor_sub = rospy.Subscriber(
            f"/{prefix}/aqdata", AQI, self.sensor_callback
        )

        rospy.logwarn(f"Started {rospy.get_name()}")

    def timeline_callback(self, req):
        res = list(
            map(
                record_to_count_stamped,
                self.traffic_collection.aggregate(
                    [
                        {"$match": {"route": req.route}},
                        {"$sort": {"stamp": -1}},
                        {"$limit": 10000},
                    ]
                ),
            )
        )
        rospy.logwarn(f"Timeline: Found {len(res)} records")
        return TimelineResponse(timeline=res)

    def last_count_callback(self, req):
        res = self.traffic_collection.aggregate(
            [{"$match": {"route": req.route}}, {"$sort": {"stamp": -1}}, {"$limit": 1}]
        )
        return TimelineResponse(timeline=[record_to_count_stamped(res.next())])

    def sensor_data_callback(self, req):
        res = self.sensor_collection.aggregate(
            [{"$match": {"route": req.route}}, {"$sort": {"stamp": -1}}]
        )
        return TimelineResponse(list(res))

    def traffic_callback(self, msg):
        self.traffic_collection.insert_one(
            {
                "stamp": msg.header.stamp.to_sec(),
                "total": msg.count.total,
                "counts": {
                    "cars": msg.count.cars,
                    "trucks": msg.count.trucks,
                    "buses": msg.count.buses,
                    "motorcycles": msg.count.motorcycles,
                },
                # TODO: get route from somewhere
                "route": 0,
                "camera": 0,
            }
        )

    def sensor_callback(self, msg):
        self.traffic_collection.insert_one(
            {
                "stamp": msg.header.stamp.to_sec(),
                "sensor": {
                    "pm25": msg.pm25,
                    "pm10": msg.pm10,
                    "tmp": msg.temp,
                    "hum": msg.hum,
                    "co2": msg.co2,
                    "pm100": msg.pm100,
                    "pm50": msg.pm50,
                },
                # TODO: get route from somewhere
                "route": 0,
            }
        )

    def close(self):
        self.client.close()

    def __del__(self) -> None:
        if hasattr(self, "client"):
            self.client.close()


if __name__ == "__main__":
    archive = MissionArchives()
    rospy.spin()
