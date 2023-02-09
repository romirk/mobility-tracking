#!/usr/bin/env python3
from __future__ import annotations

from concurrent.futures import Future, ProcessPoolExecutor
from typing import Tuple

import numpy as np
import rospy
import message_filters
from mission_control.msg import Counts, Sync
from sensorbox.msg import BoxArray
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from yolov7_package import Yolov7Detector
from yolov7_package.model_utils import coco_names

WHITELIST = ["car", "truck", "bus", "motorbike", "bicycle"]
WHITELIST_IDX = [coco_names.index(c) for c in WHITELIST]
CONFIDENCE_THRESHOLD = 0


class YoloServer:
    def __init__(self) -> None:
        rospy.init_node("detector")
        self.yolo = Yolov7Detector()
        self.counts = Counts(0, 0, 0, 0, 0, 0)

        self.multiprocessing = False

        if self.multiprocessing:
            self.executor = ProcessPoolExecutor(max_workers=5)
            self.tasks = {}

        self.pub = rospy.Publisher("/sbx/result", Counts, queue_size=10)
        if self.multiprocessing:
            self.sub = rospy.Subscriber(
                "/sbx/detect", Detection2D, self.detect_callback
            )
        else:
            self.sub = rospy.Subscriber("/sbx/detect", Detection2D, self.exec_callback)

        # sync
        self.image_sub = message_filters.Subscriber(
            "/sbx/camera/image_raw/compressed", Image
        )
        self.bound_sub = message_filters.Subscriber("/sbx/bounds", BoxArray)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.bound_sub], 10)
        self.ts.registerCallback(self.sync_callback)
        self.sync_pub = rospy.Publisher("/sbx/sync", Sync, queue_size=10)

    def __del__(self) -> None:
        if self.multiprocessing:
            self.executor.shutdown()

    def sync_callback(self, image: Image, boxes: BoxArray) -> None:
        self.sync_pub.publish(Sync(image, boxes))

    def nearest_box(
        self, boxes: list, box: list
    ) -> Tuple[int, float] | Tuple[None, float]:
        if boxes is None or len(boxes) == 0 or box is None:
            return None, float("inf")
        if len(boxes) == 1:
            return boxes[0], (boxes[0][0] - box[0]) ** 2 + (boxes[0][1] - box[1]) ** 2

        nearest = 0
        min_dist = float("inf")
        for i, b in enumerate(boxes):
            dist = (b[0] - box[0]) ** 2 + (b[1] - box[1]) ** 2
            if dist < min_dist:
                min_dist = dist
                nearest = i
        return nearest, min_dist

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
            self.pub.publish(Counts(*future.result()))

    def exec_callback(self, msg: Detection2D) -> tuple[int, int, int, int, int, int]:
        total = self.counts.total + 1
        box = msg.bbox
        frame = msg.source_img
        frame = np.frombuffer(frame.data, dtype=np.uint8).reshape(
            frame.height, frame.width, -1
        )
        r = self.yolo.detect(frame)
        if not self.multiprocessing:
            rospy.logwarn(f"Received detection {r}")
        dets = [
            d
            for d in zip(r[0][0], r[1][0], r[2][0])
            if d[0] in WHITELIST_IDX and d[2] > CONFIDENCE_THRESHOLD
        ]

        boxes = [d[1] for d in dets]
        nearest, _ = self.nearest_box(boxes, [box.center.x, box.center.y])

        nearest_class: int = dets[nearest][0] if nearest is not None else -1
        cars = self.counts.cars + (nearest_class == WHITELIST_IDX[0])
        trucks = self.counts.trucks + (nearest_class == WHITELIST_IDX[1])
        buses = self.counts.buses + (nearest_class == WHITELIST_IDX[2])
        motorcycles = self.counts.motorcycles + (nearest_class == WHITELIST_IDX[3])
        bicycles = self.counts.bicycles + (nearest_class == WHITELIST_IDX[4])

        if nearest_class == -1:
            rospy.logwarn("Sensorbox reports detection, but no vehicle was detected.")

        if not self.multiprocessing:
            self.counts = Counts(total, cars, trucks, buses, motorcycles, bicycles)
            self.pub.publish(self.counts)

        return total, cars, trucks, buses, motorcycles, bicycles


if __name__ == "__main__":
    server = YoloServer()
    rospy.spin()
