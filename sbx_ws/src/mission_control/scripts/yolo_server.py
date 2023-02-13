#!/usr/bin/env python3
from __future__ import annotations

import os
import sqlite3 as sl
from concurrent.futures import Future, ProcessPoolExecutor
from datetime import datetime
from threading import Lock

import cv2
import numpy as np
import rospy
from mission_control.msg import Counts, CountStamped
from mission_control.srv import Timeline, TimelineResponse
from vision_msgs.msg import Detection2D
from yolov7_package import Yolov7Detector
from yolov7_package.model_utils import coco_names

WHITELIST = ["car", "truck", "bus", "motorbike", "bicycle"]
WHITELIST_IDX = [coco_names.index(c) for c in WHITELIST]
CONFIDENCE_THRESHOLD = 0  # TODO: set this to 0.5
ROOT = os.path.dirname(os.path.abspath(__file__))


class YoloServer:
    def __init__(self) -> None:
        rospy.init_node("detector")
        self.frame_width = rospy.get_param("/sbx/video_width", 640)
        self.frame_height = rospy.get_param("/sbx/video_height", 480)

        rospy.logwarn(f"Starting {rospy.get_name()} node")
        rospy.logwarn(f"Whitelist: {WHITELIST} ({WHITELIST_IDX})")

        self.multiprocessing = False

        self.yolo = Yolov7Detector(traced=True)
        self.counts = Counts(0, 0, 0, 0, 0, 0)

        self.timeline_srv = rospy.ServiceProxy("/sbx/last_count", Counts)
        self.timeline_srv.wait_for_service()

        # load last count
        self.counts = self.load_last_count()

        # self.con.close()

        rospy.loginfo(f"Loaded last count: {self.counts.total}")

        if self.multiprocessing:
            self.executor = ProcessPoolExecutor(max_workers=5)
            self.tasks = {}

        self.pub = rospy.Publisher("/sbx/result", Counts, queue_size=10, latch=True)
        self.pub.publish(self.counts)
        self.sub = rospy.Subscriber(
            "/sbx/detect",
            Detection2D,
            self.detect_callback if self.multiprocessing else self.exec_callback,
        )

    def __del__(self) -> None:
        if self.multiprocessing:
            self.executor.shutdown()

    def load_last_count(self) -> Counts:
        return self.timeline_srv()

    def exec_callback(self, msg: Detection2D) -> tuple[int, int, int, int, int, int]:
        box = msg.bbox
        direction = "backward" if box.center.theta else "forward"
        frame = msg.source_img

        rospy.loginfo(
            f"Detected object moving {direction} at {box.center.x}, {box.center.y}"
        )

        img = cv2.cvtColor(
            np.frombuffer(frame.data, dtype=np.uint8).reshape(
                frame.height, frame.width, -1
            ),
            cv2.COLOR_RGB2BGR,
        )
        scale = frame.width / self.frame_width, frame.height / self.frame_height
        x, y, w, h = (
            (box.center.x - box.size_x / 2) * scale[0],
            (box.center.y - box.size_y / 2) * scale[1],
            (box.size_x) * scale[0],
            (box.size_y) * scale[1],
        )
        cx, cy = x + w / 2, y + h / 2

        # cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)

        r = self.yolo.detect(img)
        dets = [
            d
            for d in zip(r[0][0], r[1][0], r[2][0])
            if d[0] in WHITELIST_IDX  # and d[2] > CONFIDENCE_THRESHOLD
        ]
        rospy.loginfo(f"Filtered detections: {dets}")

        if len(dets) == 0:
            rospy.logwarn(
                "Sensorbox reports a detection, but YOLO did not detect a vehicle"
            )
            result = (
                self.counts.total + 1,
                self.counts.cars,
                self.counts.trucks,
                self.counts.buses,
                self.counts.motorcycles,
                self.counts.bicycles,
            )
        else:
            boxes = [(d[1][0] - cx) ** 2 + (d[1][1] - cy) ** 2 for d in dets]
            nearest = np.argmin(boxes)
            rospy.loginfo(f"Nearest box: {nearest}")
            nearest_class: int = dets[nearest][0]
            result = (
                self.counts.total + 1,
                self.counts.cars + (nearest_class == 2),
                self.counts.trucks + (nearest_class == 7),
                self.counts.buses + (nearest_class == 5),
                self.counts.motorcycles + (nearest_class == 4),
                self.counts.bicycles + (nearest_class == 1),
            )
            rospy.loginfo(
                f"Detected {coco_names[nearest_class]} at {box.center.x}, {box.center.y}"
            )

        if not self.multiprocessing:
            self.counts = Counts(*result)
            self.pub.publish(self.counts)

            cv2.imwrite(
                f"{ROOT}/out/{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}_{self.counts.total:04}.png",
                img,
            )

        return result

    def detect_callback(self, msg: Detection2D) -> None:
        rospy.logwarn("Received detection")
        future = self.executor.submit(self.exec_callback, msg)
        self.tasks[future] = msg
        future.add_done_callback(lambda f: self.done_callback)

    def done_callback(self, future: Future) -> None:
        print("Done")
        rospy.logwarn("Done")
        msg = self.tasks[future]
        del self.tasks[future]
        if future.exception():
            rospy.logerr(f"Exception in task: {future.exception()}")
        else:
            result = future.result()
            self.pub.publish(Counts(*result))


if __name__ == "__main__":
    server = YoloServer()
    rospy.spin()
