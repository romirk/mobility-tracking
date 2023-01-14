"""
Traffic Counting
"""
import datetime
import socket
import time
from multiprocessing import Value
from threading import Thread

import cv2
import imutils
import numpy as np
import requests

from utils import encode64, rs_pipeline_setup, countdown


def req_thread(url, data):
    try:
        requests.post(url, json=data)
    except Exception as e:
        print("Error in request thread: ", e)


class TrafficCounter(object):
    """
    Traffic Counter class.
    """

    def __init__(self, config):
        self.name = __name__
        self.crop_rect = []  # stores the click coordinates where to crop the frame
        self.mask_points = (
            []
        )  # stores the click coordinates of the mask to apply to cropped frame
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.p1_count_line = None
        self.p2_count_line = None
        self.counter = 0
        self.line_direction = config.direction[0]
        self.line_position = float(config.direction[1])
        self.min_area = config.min_area
        self.num_contours = config.num_contours
        self.starting_frame = config.starting_frame
        self.pipeline, self.pipeline_config = rs_pipeline_setup(config.video_width, 480, 30)

        self._vid_width = config.video_width
        self._vid_height = None  # PLACEHOLDER
        self.black_mask = (
            None  # PLACEHOLDER, user creates it by clicking on several points
        )

        self.prev_centroids = (
            []
        )  # this will contain the coordinates of the centers in the previous

        self.server = config.server
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.server, 9999))
        self.sf = self.socket.makefile("wb")

        self.record = config.record

        self.visualize = config.visual

        print("[Camera] connected to server")

        # Getting frame dimensions
        self._compute_frame_dimensions()
        self._set_up_line(config.direction[0], float(config.direction[1]))

        if config.record:
            filename = f"output/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.mp4"
            self.out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 30,
                                       (self._vid_width, self._vid_height))
            print(f"Recording to file {filename} ({self._vid_width}x{self._vid_height})")

    def _set_up_line(self, line_direction, line_position):
        if line_direction.upper() == "H" or line_direction is None:
            fract = int(self._vid_height * float(line_position))
            self.p1_count_line = (0, fract)
            self.p2_count_line = (self._vid_width, fract)
        elif line_direction.upper() == "V":
            fract = int(self._vid_width * float(line_position))
            self.p1_count_line = (fract, 0)
            self.p2_count_line = (fract, self._vid_height)
        else:
            raise ValueError('Expected an "H" or a "V" only for line direction')

    def _compute_frame_dimensions(self):
        frame = self.pipeline.wait_for_frames().get_color_frame()
        img = imutils.resize(np.asanyarray(frame.get_data()), width=self._vid_width)
        self._vid_height = img.shape[0]
        self._vid_width = img.shape[1]

    def _draw_bounding_boxes(
            self, frame, contour_id, bounding_points, cx, cy, prev_cx, prev_cy
    ):
        cv2.drawContours(frame, [bounding_points], 0, (0, 255, 0), 1)
        cv2.line(
            frame, (prev_cx, prev_cy), (cx, cy), (0, 0, 255), 1
        )  # line between last position and current position
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), 4)
        cv2.putText(
            frame, str(contour_id), (cx, cy - 15), self.font, 0.4, (255, 0, 0), 2
        )

    def __remote_update(self, frame: np.ndarray, rect: np.ndarray, dir: str, cnt: int):
        thread = Thread(target=req_thread, args=(f"http://{self.server}:5000/detect", {
            "frame": encode64(frame),
            "count": cnt,
            "rect": encode64(rect),
            "direction": dir,
            "T": str(datetime.datetime.now())
        }))
        thread.start()

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

    def bind_objects(self, frame, thresh_img):
        """Draws bounding boxes and detects when cars are crossing the line frame: numpy image where boxes will be 
        drawn onto thresh_img: numpy image after subtracting the background and all thresholds and noise reduction 
        operations are applied
        """
        orig_frame = frame.copy()
        contours, _ = cv2.findContours(
            thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )  # this line is for opencv 2.4, and also now for OpenCV 4.4, so this is the current one
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[: self.num_contours]

        cnt_id = 1
        cur_centroids = []
        for c in contours:
            if (
                    cv2.contourArea(c) < self.min_area
            ):  # ignore contours that are smaller than this area
                continue
            rect = cv2.minAreaRect(c)
            bounding = cv2.boundingRect(c)
            points = cv2.boxPoints(rect)  # This is the way to do it in opencv 3.1
            points = np.int0(points)

            # Getting the center coordinates of the contour box
            cx = int(rect[0][0])
            cy = int(rect[0][1])

            if cy > 240:
                continue

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

            _is_crossed, direction = self._is_line_crossed(frame, cx, cy, prev_cx, prev_cy)
            if _is_crossed:
                self.__remote_update(orig_frame, bounding, direction, self.counter)
                print(f"\r{self.counter}", end="")
            self._draw_bounding_boxes(frame, cnt_id, points, cx, cy, prev_cx, prev_cy)

            cv2.line(frame, (0, 240), (self._vid_width, 240), (0, 0, 255), 1)

            cnt_id += 1
        self.prev_centroids = cur_centroids  # updating centroids for next frame

    def _set_up_masks(self):
        frame = self.pipeline.wait_for_frames().get_color_frame()
        img = cv2.resize(np.asanyarray(frame.get_data()), (self._vid_width, self._vid_height))

        self.raw_avg = np.float32(img)
        self.raw_avg = cv2.resize(self.raw_avg, (self._vid_width, self._vid_height))

        print("Ready. Starting in")
        countdown(5)

    def main_loop(self, running: Value):
        self._set_up_masks()
        rate_of_influence = 0.01
        print("running")

        try:
            while running.value:
                t0 = time.time()
                frame = self.pipeline.wait_for_frames().get_color_frame()
                frame_id = int(frame.frame_number)  # get current frame index
                img = cv2.resize(np.asanyarray(frame.get_data()), (self._vid_width, self._vid_height))

                if not self.visualize:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    self.socket.send(img.tobytes())

                working_img = img.copy()

                cv2.accumulateWeighted(working_img, self.raw_avg, rate_of_influence)

                if frame_id < self.starting_frame:
                    print(self.raw_avg)
                    continue

                background_avg = cv2.convertScaleAbs(
                    self.raw_avg
                )  # reference background average image

                subtracted_img = self.subtract(background_avg, working_img)

                # Adding extra blur
                subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)
                subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)
                subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)
                subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)

                # Applying threshold
                _, threshold_img = cv2.threshold(subtracted_img, 30, 255, 0)

                # Noise Reduction
                dilated_img = cv2.dilate(threshold_img, None)
                dilated_img = cv2.dilate(dilated_img, None)
                dilated_img = cv2.dilate(dilated_img, None)
                dilated_img = cv2.dilate(dilated_img, None)
                dilated_img = cv2.dilate(dilated_img, None)

                # Drawing bounding boxes and counting
                self.bind_objects(img, dilated_img)

                if self.visualize:
                    self.socket.send(img.tobytes())

                if self.record:
                    self.out.write(img)

                t1 = time.time()
                # print(f"\r{frame_id} {1 / (t1 - t0)}", end="")
        except KeyboardInterrupt:
            pass
        except ConnectionResetError:
            print("Connection reset by peer")
        finally:
            self.pipeline.stop()
            if self.record:
                self.out.release()
            print("[Camera] Stopping counter...")

    def subtract(self, background_avg, working_img):
        return np.linalg.norm(background_avg - working_img, axis=2)
