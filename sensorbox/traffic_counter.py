"""
Traffic Counting
"""
import datetime
import socket
import time
from multiprocessing import Value, Process
from multiprocessing.shared_memory import SharedMemory
from threading import Thread

import cv2
import imutils
import numpy as np
import requests

from .utils import encode64, rs_pipeline_setup, countdown


def req_thread(url, data):
    try:
        requests.post(url, json=data)
    except Exception as e:
        print("Error in request thread: ", e)


def streamer_thread(url: str, dim: tuple, mem: str, running: Value):
    mem = SharedMemory(name=mem)
    frame = np.ndarray(dim, dtype=np.uint8, buffer=mem.buf)
    transport = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    transport.connect((url, 9999))
    print("Connected to streamer")

    try:
        while running.value:
            transport.sendall(frame)
            time.sleep(0.25)
    except ConnectionResetError:
        print("Connection reset by peer")
    except KeyboardInterrupt:
        pass
    finally:
        running.value = False
        mem.close()
        transport.close()

TEST_FILE = "./mobility-tracking/ignore/test.mp4"

class TrafficCounter(object):
    """
    Traffic Counter class.
    """

    def __init__(self, config):
        self.name = __name__
        self.running = Value("b", True)
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
        self.server = config.server
        self.prev_centroids = (
            []
        )  # this will contain the coordinates of the centers in the previous
        if not config.debug:
            self.debug = False
            self._vid_width = config.video_width
            self._vid_height = 800
            self.debug = False
            self.pipeline, self.pipeline_config = rs_pipeline_setup(self._vid_width, self._vid_height, 30)
            self._compute_frame_dimensions()
        else:
            self.debug = True
            self.cap = cv2.VideoCapture()
            self.cap.open(TEST_FILE)
            self._vid_width = int(self.cap.get(3))
            self._vid_height = int(self.cap.get(4))

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=100, varThreshold=50, detectShadows=True
        )

        self.mem = SharedMemory(create=True, size=921600, name="frame")
        self.shared_frame = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=self.mem.buf)

        self.record = config.record

        self.visualize = config.visual

        # Getting frame dimensions
        self._set_up_line(config.direction[0], float(config.direction[1]))

        if config.record:
            filename = f"output/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.mp4"
            self.out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 30,
                                       (self._vid_width, self._vid_height))
            print(f"Recording to file {filename} ({self._vid_width}x{self._vid_height})")

        self.streamer = Process(target=streamer_thread, args=(
            config.server, self.shared_frame.shape, self.mem.name, self.running), daemon=True)
        self.streamer.start()

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
        if self.debug:
            self._vid_height = int(self.cap.get(4))
        else:
            frame = self.pipeline.wait_for_frames().get_color_frame()
            img = imutils.resize(np.asanyarray(frame.get_data()), width=self._vid_width)
            self._vid_height = img.shape[0]
            self._vid_width = img.shape[1]

    def _draw_bounding_boxes(
            self, frame, contour_id, bounding_points, cx, cy, prev_cx, prev_cy, area
    ):
        cv2.drawContours(frame, [bounding_points], 0, (0, 255, 0), 1)
        cv2.line(
            frame, (prev_cx, prev_cy), (cx, cy), (0, 0, 255), 1
        )  # line between last position and current position
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), 4)
        cv2.putText(
            frame, str(contour_id) + " " + str(area), (cx, cy - 15), self.font, 0.4, (255, 0, 0), 2
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
            area = cv2.contourArea(c)

            if (
                    area < self.min_area
            ):  # ignore contours that are smaller than this area
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

            _is_crossed, direction = self._is_line_crossed(frame, cx, cy, prev_cx, prev_cy)
            if _is_crossed:
                self.__remote_update(orig_frame, bounding, direction, self.counter)
                print(f"\r{self.counter}", end="")
            self._draw_bounding_boxes(frame, cnt_id, points, cx, cy, prev_cx, prev_cy, area)

            cnt_id += 1
        self.prev_centroids = cur_centroids  # updating centroids for next frame

    def _set_up_masks(self):
        """Sets up the masks for the background subtraction and the thresholding operations"""
        
        assert self.cap.isOpened()
        if self.debug:
            _, frame = self.cap.read()
            img = frame
        else:
            frame = self.pipeline.wait_for_frames().get_color_frame()
            img = cv2.resize(np.asanyarray(frame.get_data()), (self._vid_width, self._vid_height))

        self.raw_avg = np.copy(img).astype(np.float32)
        
        print(self.raw_avg.shape)
        print("Ready. Starting in")
        countdown(5)

    def main_loop(self, running: Value):
        self._set_up_masks()
        rate_of_influence = 0.01
        print("running")

        try:
            i = 0
            while running.value:
                t0 = time.time()
                if self.debug:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
                    ret, frame = self.cap.read()
                    if not ret: continue
                    frame_id = i
                    i += 1
                    img = frame
                else:
                    frame = self.pipeline.wait_for_frames().get_color_frame()
                    frame_id = int(frame.frame_number)  # get current frame index
                    img = cv2.resize(np.asanyarray(frame.get_data()), (self._vid_width, self._vid_height))

                working_img = img.copy()
                cv2.accumulateWeighted(working_img, self.raw_avg, rate_of_influence)

                if frame_id < self.starting_frame:
                    # print(self.raw_avg)
                    continue

                background_avg = cv2.convertScaleAbs(
                    self.raw_avg
                )  # reference background average image

                # Background subtraction
                fg_mask = self.bg_subtractor.apply(working_img, background_avg, rate_of_influence)
                blurred = cv2.GaussianBlur(fg_mask, (31, 31), 0)
                ret1, th1 = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

                kernel = np.ones((7, 7), np.uint8)
                dilated = cv2.dilate(th1, kernel)
                final_img = cv2.dilate(dilated, None)

                # Drawing bounding boxes and counting
                t2 = time.time()
                self.bind_objects(img, final_img)
                # sys.stdout.write(f"\rbound objects in {time.time() - t2} at frame")

                # if self.debug:
                #     frame_1 = cv2.cvtColor(fg_mask, cv2.COLOR_GRAY2BGR)
                #     frame_2 = cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)
                #     frame_3 = cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR)
                #     frame_4 = img
                #
                #     frame_12 = np.hstack((frame_1, frame_2))
                #     frame_34 = np.hstack((frame_3, frame_4))
                #     final_img = np.vstack((frame_12, frame_34))

                # Displaying the frame
                # colored_final_img = cv2.cvtColor(final_img, cv2.COLOR_GRAY2BGR)
                # colored_final_img = final_img
                colored_final_img = img

                down_sampled = cv2.resize(colored_final_img, (640, 480))
                np.copyto(self.shared_frame, down_sampled)

                if self.record:
                    self.out.write(img)

                t1 = time.time()
                # sys.stdout.write(f" {frame_id} {1 / (t1 - t0)}")
        except KeyboardInterrupt:
            pass
        finally:
            self.running.value = False
            self.mem.close()
            if not self.debug:
                self.pipeline.stop()
            if self.record:
                self.out.release()
            if self.debug:
                self.cap.release()
            print("[Camera] Stopping counter...")

    def subtract(self, background_avg, working_img):
        return np.linalg.norm(background_avg - working_img, axis=2)
