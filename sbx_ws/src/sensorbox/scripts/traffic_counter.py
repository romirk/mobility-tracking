#!/usr/bin/env python3

"""
Mobility Tracking

Copyright (C) 2023 YES!Delft
by Romir Kulshreshtha and Anthony Nikolaidis
"""

# from __future__ import annotations

from math import pi
from multiprocessing import Event

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Polygon, Pose2D
from sensor_msgs.msg import CompressedImage, Image
from sensorbox.msg import AnnotatedImage
from vision_msgs.msg import BoundingBox2D, Detection2D
from std_msgs.msg import Float32
from typing import Tuple


class MobilityTracker:
    """
    Mobility Tracker

    This class is responsible for detecting and tracking objects in a video stream.
    For the 2023 YES!Delft Region of the Future project.
    """

    def __init__(self):
        rospy.init_node("mt")
        self.prefix = rospy.get_param("prefix")
        self.camera_name = rospy.get_param("camera_name")
        theta = rospy.get_param("theta", 0)  # degrees
        line_posistion = rospy.get_param("line_position", 0.5)
        self.min_area = rospy.get_param("min_area", 5000)
        self.num_contours = rospy.get_param("num_contours", 10)
        self.starting_frame = rospy.get_param("starting_frame", 30)
        self.fps = rospy.get_param("fps", 30)
        self._vid_width = rospy.get_param("video_width", 640)
        self._vid_height = rospy.get_param("video_height", 480)
        self.debug_view = rospy.get_param("debug_view", False)

        rospy.logwarn(f"Starting {self.prefix}/counter node")
        if self.debug_view:
            rospy.logwarn("Debug view on")

        self.stopped = Event()

        self.rate_of_influence = 0.01
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.p1_count_line: tuple = ()
        self.p2_count_line: tuple = ()
        self.counter = 0
        self.mask: Polygon = Polygon()
        self.mask_img: np.ndarray = np.zeros(
            (self._vid_height, self._vid_width, 3), np.uint8
        )
        self.raw_avg = np.zeros((self._vid_height, self._vid_width, 3))
        self.prev_centroids = (
            []
        )  # this will contain the coordinates of the centers in the previous

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=100, varThreshold=50, detectShadows=True
        )

        self._set_up_line(theta, line_posistion)
        self._set_up_mask()

        # data topics
        self.frame_sub = rospy.Subscriber(
            f"/{self.camera_name}/color/image_raw", Image, self.frame_callback
        )
        self.detection_pub = rospy.Publisher(
            f"/{self.prefix}/detect", Detection2D, queue_size=10
        )
        self.animg_pub = rospy.Publisher(
            f"/{self.prefix}/bboxed", AnnotatedImage, queue_size=10
        )

        # control topics
        self.mask_sub = rospy.Subscriber(f"/{self.prefix}/mask", Polygon, self.set_mask)
        self.line_sub = rospy.Subscriber(f"/{self.prefix}/line", Pose2D, self.set_line)
        self.roi_sub = rospy.Subscriber(f"/{self.prefix}/roi", Float32, self.set_roi)
        self.area_sub = rospy.Subscriber(
            f"/{self.prefix}/area", Float32, self.set_min_area
        )

    def _set_up_line(self, theta, position):
        """
        Defines the line that will be used to count the number of objects
        """
        if theta > 90 or theta < -90:
            raise ValueError("Theta must be between -90 and 90")
        if position > 1 or position < 0:
            raise ValueError("Position must be between 0 and 1")

        if theta == 0:
            self.p1_count_line = (0, self._vid_height * position)
            self.p2_count_line = (self._vid_width, self._vid_height * position)

        elif theta == 90 or theta == -90:
            self.p1_count_line = (self._vid_width * position, 0)
            self.p2_count_line = (self._vid_width * position, self._vid_height)

        else:
            m = np.tan(theta * pi / 180)
            b = self._vid_height * position - m * self._vid_width * position
            self.p1_count_line = (0, b)
            self.p2_count_line = (self._vid_width, m * self._vid_width + b)

    def _set_up_mask(self):
        """
        Sets up the mask for the video stream
        """
        if self.mask.points:
            self.mask_img = np.zeros((self._vid_height, self._vid_width, 3), np.uint8)
            points = np.array(self.mask.points, np.int32)
            rospy.logwarn(f"Mask: {points}")
            cv2.fillPoly(self.mask_img, points, (255, 255, 255))
        else:
            self.mask_img = (
                np.ones((self._vid_height, self._vid_width, 3), np.uint8) * 255
            )

    def _distance_from_line(self, cx, cy):
        """
        Calculates the distance of a point from a line
        """
        x1, y1 = self.p1_count_line
        x2, y2 = self.p2_count_line
        return (cx - x1) * (y2 - y1) - (cy - y1) * (x2 - x1)

    def _is_line_crossed(self, cx, cy, prev_cx, prev_cy) -> Tuple[bool, bool]:
        """
        Checks if a line is crossed
        """
        if cx == prev_cx and cy == prev_cy:
            return False, False
        d1 = self._distance_from_line(cx, cy)
        d2 = self._distance_from_line(prev_cx, prev_cy)
        return d1 * d2 < 0, d1 > 0

    def bind_objects(self, incoming_frame: Image, thresh_img):
        """
        Draws bounding boxes and detects when cars are crossing the line frame: numpy image where boxes will be
        drawn onto thresh_img: numpy image after subtracting the background and all thresholds and noise reduction
        operations are applied
        """
        contours, _ = cv2.findContours(
            thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )  # this line is for opencv 2.4, and also now for OpenCV 4.4, so this is the current one
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[
            : self.num_contours
        ]

        cur_centroids = []
        boxes: list[BoundingBox2D] = []

        for c in contours:
            area = cv2.contourArea(c)

            if area < self.min_area:  # ignore contours that are smaller than this area
                continue

            rect = cv2.minAreaRect(c)
            bounding = cv2.boundingRect(c)
            points = cv2.boxPoints(rect)  # This is the way to do it in opencv 3.1
            points = np.intp(points)

            # Getting the center coordinates of the contour box
            cx = int(rect[0][0])
            cy = int(rect[0][1])

            # if cy > 240:
            #     continue

            w, h = rect[1]  # Unpacks the width and height of the frame

            cs = np.array((cx, cy))
            cur_centroids.append((cx, cy))

            # Finding the centroid of c in the previous frame
            if len(self.prev_centroids) == 0:
                prev_cx, prev_cy = cx, cy
            elif len(contours) == 0:
                prev_cx, prev_cy = cx, cy
            else:
                min_point = None, None
                min_dist = None
                for i in range(len(self.prev_centroids)):
                    dist = np.linalg.norm(cs - self.prev_centroids[i])
                    if (min_dist is None) or (dist < min_dist):
                        min_dist = dist
                        min_point = self.prev_centroids[i]

                if min_dist < 1.5 * w / 2:
                    prev_cx, prev_cy = min_point
                else:
                    prev_cx, prev_cy = cx, cy

            crossed, direction = self._is_line_crossed(cx, cy, prev_cx, prev_cy)
            if crossed:
                self.detection_pub.publish(
                    Detection2D(
                        header=incoming_frame.header,
                        source_img=incoming_frame,
                        bbox=BoundingBox2D(
                            center=Pose2D(x=cx, y=cy, theta=direction),
                            size_x=w,
                            size_y=h,
                        ),
                    )
                )

            boxes.append(
                BoundingBox2D(
                    center=Pose2D(
                        x=bounding[0] + bounding[2] / 2, y=bounding[1] + bounding[3] / 2
                    ),
                    size_x=bounding[2],
                    size_y=bounding[3],
                )
            )

        self.prev_centroids = cur_centroids

        return boxes

    def frame_callback(self, frame: Image):
        """
        Main callback for the video stream. This is where the magic happens(?)
        """
        height = frame.height
        width = frame.width
        data = np.frombuffer(frame.data, dtype=np.uint8).reshape(height, width, -1)
        frame_id = frame.header.seq  # get current frame index
        img = cv2.resize(data, (self._vid_width, self._vid_height))

        working_img = img.copy()

        if self.mask.points:
            rospy.logwarn_once("Masks enabled")
            working_img = cv2.bitwise_and(working_img, working_img, mask=self.mask_img)

        cv2.accumulateWeighted(working_img, self.raw_avg, self.rate_of_influence)
        if frame_id < self.starting_frame:
            return

        background_avg = cv2.convertScaleAbs(
            self.raw_avg
        )  # reference background average image

        # Background subtraction
        fg_mask = self.bg_subtractor.apply(
            working_img, background_avg, self.rate_of_influence
        )
        blurred = cv2.GaussianBlur(fg_mask, (31, 31), 0)
        _, th1 = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

        kernel = np.ones((7, 7), np.uint8)
        dilated = cv2.dilate(th1, kernel, iterations=3)
        final_img = cv2.dilate(dilated, None)

        boxes = self.bind_objects(frame, final_img)

        annotated_img = AnnotatedImage(
            header=rospy.Header(stamp=rospy.Time.now()),
            img=CompressedImage(
                header=frame.header,
                format="jpeg",
                data=cv2.imencode(".jpg", final_img if self.debug_view else img)[
                    1
                ].tostring(),
            ),
            boxes=boxes,
        )
        self.animg_pub.publish(annotated_img)

    def set_mask(self, mask: Polygon):
        self.mask = mask
        self._set_up_mask()

    def set_roi(self, roi: Float32):
        self.rate_of_influence = roi.data

    def set_min_area(self, min_area: Float32):
        self.min_area = min_area.data

    def set_line(self, line: Pose2D):
        self._set_up_line(line.theta, line.y)


if __name__ == "__main__":
    t = MobilityTracker()
    rospy.spin()
