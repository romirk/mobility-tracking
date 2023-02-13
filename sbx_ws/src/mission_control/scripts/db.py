import pymongo
import rospy
from mission_control.srv import Timeline, TimelineResponse
from sensorbox.msg import AQI
from vision_msgs.msg import Detection2D

# Replace the uri string with your MongoDB deployment's connection string.
__CONN_STR = "mongodb://localhost:27017"


class MissionArchives:
    def __init__(self) -> None:
        rospy.init_node("time-machine")
        prefix = rospy.get_param("prefix", "sbx")

        self.client = pymongo.MongoClient(__CONN_STR)
        self.db = self.client.impactlab
        self.collection = self.db.mobility_tracking

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

    def timeline_callback(self, req):
        route = req.route
        res = self.collection.find(
            {"timestamp": {"$gte": req.start, "$lte": req.end}, "route": route},
            {"_id": 0},
        )
        # rospy.logwarn(f"Found {res.count()} records")
        return TimelineResponse(list(res))

    def last_count_callback(self, req):
        res = self.collection.aggregate(
            [
                {"$sort": {"timestamp": -1}, "route": req.route},
                {
                    "$group": {
                        "_id": "$sensor",
                        "timestamp": {"$first": "$timestamp"},
                        "count": {"$first": "$count"},
                    }
                },
            ]
        )
        return TimelineResponse(list(res))

    def sensor_data_callback(self, req):
        res = self.collection.find(
            {"timestamp": {"$gte": req.start, "$lte": req.end}, "sensor": req.sensor},
            {"_id": 0},
        )
        return TimelineResponse(list(res))

    def traffic_callback(self, msg):
        self.collection.insert_one(
            {
                "timestamp": msg.header.stamp.to_sec(),
                "sensor": "traffic",
                "count": msg.count.total,
            }
        )

    def sensor_callback(self, msg):
        self.collection.insert_one(
            {
                "timestamp": msg.header.stamp.to_sec(),
                "sensor": {
                    "pm25": msg.pm25,
                    "pm10": msg.pm10,
                    "tmp": msg.temp,
                    "hum": msg.hum,
                    "co2": msg.co2,
                    "pm100": msg.pm100,
                },
            }
        )

    def __del__(self) -> None:
        self.client.close()
