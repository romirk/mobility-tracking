#!/usr/bin/env python3

"""
Traffic Counting
"""
import time
from multiprocessing import Event

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from ros_numpy import msgify, numpify

# from jetson_inference import detectNet


# TEST_FILE = "./mobility-tracking/ignore/test.mp4"
TEST_FILE = "./test.mp4"


class TrafficCounter:
    """
    Traffic Counter class.
    """

    def __init__(self):
        self.name = __name__
        rospy.init_node(self.name)
        self.stopped = Event()
        self.crop_rect = []  # stores the click coordinates where to crop the frame
        self.mask_points = (
            []
        )  # stores the click coordinates of the mask to apply to cropped frame
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.p1_count_line: tuple = ()
        self.p2_count_line: tuple = ()
        self.counter = 0
        self.line_direction = rospy.get_param("~direction", "H")
        self.line_position = rospy.get_param("~line_position", 0.5)
        self.min_area = rospy.get_param("~min_area", 5000)
        self.num_contours = rospy.get_param("~num_contours", 10)
        self.starting_frame = rospy.get_param("~starting_frame", 30)
        self.prev_centroids = (
            []
        )  # this will contain the coordinates of the centers in the previous
        self.fps = rospy.get_param("~fps", 30)

        self._vid_width = rospy.get_param("~video_width", 640)
        self._vid_height = rospy.get_param("~video_height", 480)

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=100, varThreshold=50, detectShadows=True
        )

        self._set_up_line(self.line_direction, self.line_position)

        self.rate_of_influence = 0.01
        self.raw_avg = np.zeros((self._vid_height, self._vid_width, 3))

        self.frame_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.frame_callback
        )
        self.detection_pub = rospy.Publisher("/detect/raw", Detection2D, queue_size=10)

    def _set_up_line(self, line_direction, line_position):
        if line_direction.upper() == "H" or line_direction is None:
            fract = int(self._vid_height * float(line_position))
            self.p1_count_line = (0, fract)
            self.p2_count_line = (self._vid_width, fract)
        elif line_direction.upper() == "V":
            fract = int(self._vid_width * float(line_position))
            self.p1_count_line = (fract, 0)
            self.p2_count_line = (fract, self._vid_height)
            print(f"vertical line at {fract}")
        else:
            raise ValueError('Expected an "H" or a "V" only for line direction')

    def _draw_bounding_boxes(
        self, frame, contour_id, bounding_points, cx, cy, prev_cx, prev_cy, area
    ):
        cv2.drawContours(frame, [bounding_points], 0, (0, 255, 0), 1)
        cv2.line(
            frame, (prev_cx, prev_cy), (cx, cy), (0, 0, 255), 1
        )  # line between last position and current position
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), 4)
        cv2.putText(
            frame,
            str(contour_id) + " " + str(area),
            (cx, cy - 15),
            self.font,
            0.4,
            (255, 0, 0),
            2,
        )

    def _is_line_crossed(self, frame, cx, cy, prev_cx, prev_cy):
        if self.line_direction.upper() == "H":
            if (prev_cy <= self.p1_count_line[1] <= cy) or (
                cy <= self.p1_count_line[1] <= prev_cy
            ):
                self.counter += 1
                cv2.line(frame, self.p1_count_line, self.p2_count_line, (0, 255, 0), 5)
                return True, "u" if cy - self.p1_count_line[1] < 0 else "d"
        elif self.line_direction.upper() == "V":
            if (prev_cx <= self.p1_count_line[0] <= cx) or (
                cx <= self.p1_count_line[0] <= prev_cx
            ):
                self.counter += 1
                cv2.line(frame, self.p1_count_line, self.p2_count_line, (0, 255, 0), 5)
                return True, "l" if cx - self.p1_count_line[0] < 0 else "r"
        return False, None

    # def classify(self, frame, bounding_box):
    #     detections = self.net.Detect(frame, overlay="box,labels,conf")

    def bind_objects(self, frame, thresh_img):
        """Draws bounding boxes and detects when cars are crossing the line frame: numpy image where boxes will be
        drawn onto thresh_img: numpy image after subtracting the background and all thresholds and noise reduction
        operations are applied
        """
        orig_frame = frame.copy()
        contours, _ = cv2.findContours(
            thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )  # this line is for opencv 2.4, and also now for OpenCV 4.4, so this is the current one
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[
            : self.num_contours
        ]

        cnt_id = 1
        cur_centroids = []
        for c in contours:
            area = cv2.contourArea(c)

            if area < self.min_area:  # ignore contours that are smaller than this area
                continue
            rect = cv2.minAreaRect(c)
            bounding = cv2.boundingRect(c)
            points = cv2.boxPoints(rect)  # This is the way to do it in opencv 3.1
            points = np.int0(points)

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
                min_point = None
                min_dist = None
                for i in range(len(self.prev_centroids)):
                    dist = np.linalg.norm(
                        cs - self.prev_centroids[i]
                    )  # numpy's way to find the euclidean distance between two points
                    if (min_dist is None) or (dist < min_dist):
                        min_dist = dist
                        min_point = self.prev_centroids[i]
                # This if is meant to reduce over-counting errors
                if min_dist < 1.5 * w / 2:
                    prev_cx, prev_cy = min_point
                else:
                    prev_cx, prev_cy = cx, cy
                # prev_cx,prev_cy = min_point

            _is_crossed, direction = self._is_line_crossed(
                frame, cx, cy, prev_cx, prev_cy
            )
            if _is_crossed:
                detection_msg = Detection2D()
                detection_msg.header.frame_id = "camera"
                detection_msg.header.stamp = rospy.Time.now()
                detection_msg.bbox.center.x = cx
                detection_msg.bbox.center.y = cy
                detection_msg.bbox.size_x = w
                detection_msg.bbox.size_y = h
                detection_msg.source_img = msgify(Image, orig_frame, encoding="bgr8")
                self.detection_pub.publish(detection_msg)

            self._draw_bounding_boxes(
                frame, cnt_id, points, cx, cy, prev_cx, prev_cy, area
            )

            cnt_id += 1
        self.prev_centroids = cur_centroids  # updating centroids for next frame

    def frame_callback(self, frame: Image):
        height = frame.height
        width = frame.width
        data = np.frombuffer(frame.data, dtype=np.uint8).reshape(height, width, -1)
        frame_id = frame.header.seq  # get current frame index
        img = cv2.resize(data, (self._vid_width, self._vid_height))

        working_img = img.copy()

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
        ret1, th1 = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

        kernel = np.ones((7, 7), np.uint8)
        dilated = cv2.dilate(th1, kernel)
        final_img = cv2.dilate(dilated, None)

        # Drawing bounding boxes and counting
        t2 = time.time()
        self.bind_objects(img, final_img)

        # colored_final_img = img

        # down_sampled = cv2.resize(colored_final_img, (640, 480))

        # todo publish image


if __name__ == "__main__":
    t = TrafficCounter()
    rospy.spin()
