#!/usr/bin/env python3
from __future__ import annotations

from typing import Tuple

import rospy
from vision_msgs.msg import Detection2D
from mission_control.msg import Counts
from yolov7_package import Yolov7Detector
from yolov7_package.model_utils import coco_names
import numpy as np

WHITELIST = ["car", "truck", "bus", "motorbike", "bicycle"]
WHITELIST_IDX = [coco_names.index(c) for c in WHITELIST]
CONFIDENCE_THRESHOLD = 0.3


class YoloServer:
    def __init__(self) -> None:
        rospy.init_node("detector")
        self.yolo = Yolov7Detector()
        self.counts = Counts(0, 0, 0, 0, 0, 0)
        self.pub = rospy.Publisher( "/sbx/result", Counts, queue_size=10)
        self.sub = rospy.Subscriber("/sbx/detect", Detection2D, self.callback)

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

    def callback(self, msg: Detection2D) -> None:
        total = self.counts.total + 1
        box = msg.bbox
        frame = msg.source_img
        frame = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, -1)
        r = self.yolo.detect(frame)
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

        self.counts = Counts(total, cars, trucks, buses, motorcycles, bicycles)
        self.pub.publish(self.counts)


if __name__ == "__main__":
    server = YoloServer()
    rospy.spin()
