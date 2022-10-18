"""
Lane Tracking

- This program tracks traffic moving through a lane and reports the number of
vehicles that have passed through the lane in the duration of the program.
It employs an Intel RealSense D435i camera for image capture and does not
use object detection or classification. Instead, it uses a simple convolution
filter background subtraction to detect the presence of a vehicle in the lane. 

(c) 2022 YES!Delft
@author: Romir Kulshrestha <romir@imagifight.in>, Tony Nikolaidis <tony@infopelago.gr>
"""

import multiprocessing
from time import sleep
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import re

# from matplotlib import pyplot as plt
from configuration import Configuration, ConfigurationBuilder


# definitions
DEPTH_HORIZONTAL_FOV = 87
DEPTH_VERTICAL_FOV = 58
RGB_HORIZONTAL_FOV = 69
RGB_VERTICAL_FOV = 42

IMG_HEIGHT = 480
IMG_WIDTH = 640

# default assumptions
# ASSUMED_MAXIMUM_SPEED = 60  # km/h
# ASSUMED_MINIMUM_SPEED = 10  # km/h


def countdown(length):
    """Print sleep countdown"""

    for i in range(length, 0, -1):
        print(f"\r{i}", end="", flush=True)
        sleep(1)
    print(f"\r0")


class LaneTracker:
    _DEFAULT_CONFIG = {
        "alpha_t": 10,
        "buffer_size": 0,
        "min_t": 0.8247371503219356,
        "max_t": 4.948422901931615,
        "scan_w": 100,
        "scan_h": IMG_HEIGHT,
        "scan_x": -1,
        "scan_y": -1,
        "camera_h": 10,
        "fps": 30,
        "roi": 0.01,
    }

    def __init__(
        self,
        config: Configuration,
        pipeline: rs.pipeline,
        counter: multiprocessing.Value,
        barrier,
        id: int,
    ):

        print(f"Initializing lane tracker {id}...")
        self._id = id
        if type(config) is ConfigurationBuilder:
            config = config.build()
        if type(config) is not Configuration:
            raise ValueError("Invalid configuration type")
        self.pipeline = pipeline

        self.q = multiprocessing.Queue(maxsize=1)
        self.counter = counter
        self.barrier = barrier

        # args
        self.frame_rate: int = int(config.fps)
        self.camera_height: float = config.camera_h
        self.scan_width: int = int(config.scan_w)
        self.scan_height: int = int(config.scan_h)
        self.alpha_threshold: float = config.alpha_t
        self.min_time_period: float = config.min_t
        self.max_time_period: float = config.max_t
        self.rate_of_influence: float = config.roi
        self.scan_x = int(
            config.scan_x
            if config.scan_x >= 0
            else (IMG_WIDTH // 2 - self.scan_width // 2)
        )
        self.scan_y = int(
            config.scan_y if config.scan_y >= 0 else (IMG_HEIGHT - self.scan_height)
        )
        self.buffer_size = config.buffer_size or int(
            self.max_time_period * self.frame_rate * self.scan_width / IMG_WIDTH
        )

        self.spike_size = math.floor(
            self.min_time_period * self.frame_rate * self.scan_width / IMG_WIDTH
        )
        self.scan_length = (
            2 * self.camera_height * np.tan(RGB_HORIZONTAL_FOV / 2 * np.pi / 180)
        )
        self.raw_bg_avg = np.ndarray(0)
        self.raw_alpha_avg = 0
        self.max_alpha = 0

        # object counter
        self.buffer = np.asarray([0] * self.buffer_size, dtype=np.int32)
        self.b = multiprocessing.Array("B", [0] * self.buffer_size)

        self.pattern = re.compile(r"0+(1{" + str(self.spike_size) + r"}1*)0+")

        self.pipeline = pipeline
        self.setup_imgs()

        self.alphas = []

    @staticmethod
    def build_configuration(config=None):
        c = LaneTracker._DEFAULT_CONFIG.copy()
        if config is not None:
            c.update(config)
        return ConfigurationBuilder(c)

    def crop_and_grayscale(self, frame):
        return cv2.cvtColor(
            frame[
                self.scan_y : self.scan_y + self.scan_height,
                self.scan_x : self.scan_x + self.scan_width,
            ],
            cv2.COLOR_BGR2GRAY,
        )

    def setup_pipeline(self):

        # Start streaming
        self.pipeline.start(self.config)

    def setup_imgs(self):
        frames = self.pipeline.wait_for_frames()
        frame = np.asarray(frames.get_color_frame().get_data())
        color_img = self.crop_and_grayscale(frame)
        self.raw_bg_avg = np.float32(color_img)

        i = 0
        while i < 10:
            frames = self.pipeline.wait_for_frames()
            frame = np.asarray(frames.get_color_frame().get_data())
            color_img = self.crop_and_grayscale(frame)
            cv2.accumulateWeighted(color_img, self.raw_bg_avg, self.rate_of_influence)
            i += 1

    def process_img(self, frame):
        color_image = self.crop_and_grayscale(frame)

        # background subtraction taken from
        # https://github.com/andresberejnoi/ComputerVision/blob/398abab2e73b50890865a3996ebaafc0b7208074
        cv2.accumulateWeighted(color_image, self.raw_bg_avg, self.rate_of_influence)
        background_avg = cv2.convertScaleAbs(
            self.raw_bg_avg
        )  # reference background average image
        subtracted_img = cv2.absdiff(background_avg, color_image)

        # print(background_avg)

        # apply filter
        subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)
        subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)
        subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)
        subtracted_img = cv2.GaussianBlur(subtracted_img, (21, 21), 0)

        _, threshold_img = cv2.threshold(subtracted_img, 30, 255, 0)

        dilated_img = cv2.dilate(threshold_img, None)
        dilated_img = cv2.dilate(dilated_img, None)
        dilated_img = cv2.dilate(dilated_img, None)
        dilated_img = cv2.dilate(dilated_img, None)

        dilated_img = cv2.dilate(dilated_img, None)

        view_img = np.hstack(
            (
                color_image,
                background_avg,
                subtracted_img,
                threshold_img,
                dilated_img,
            )
        )

        return dilated_img, view_img

    def __call__(self):
        setup_period = False
        while True:
            try:
                try:
                    frames = self.q.get()
                except Exception as e:
                    print(f"Error reading frame in lane tracker {self._id}: {e}")
                    frames = None
                if frames is None:
                    # poison pill
                    fname = f"lane_tracker_{self._id}.txt"
                    print(f"lane {self._id} tracking stopped\n writing to file {fname}")
                    with open(fname, "w") as f:
                        f.write("\n".join(map(lambda e: str(int(e * 100)), self.alphas)))
                    self.barrier.wait()
                    break

                subtracted_img, view_img = self.process_img(frames)

                # if True:
                #     cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
                #     cv2.imshow("RealSense", view_img)

                #     if cv2.waitKey(1) & 0xFF == ord("q"):
                #         break

                alpha = np.sum(subtracted_img) / (self.scan_height * self.scan_width)

                cv2.accumulateWeighted(
                    alpha, self.raw_alpha_avg, self.rate_of_influence
                )
                alpha -= self.raw_alpha_avg

                if setup_period:
                    if alpha < 3:
                        setup_period = False
                        self.b = "--"
                    else:
                        return

                self.alphas.append(alpha)

                if alpha > self.max_alpha:
                    self.max_alpha = alpha

                self.alpha_threshold = int(self.max_alpha / 2)

                beta = 1 if alpha > self.alpha_threshold else 0

                # append to buffer
                self.buffer = np.roll(self.buffer, -1)
                self.buffer[-1] = beta

                # process buffer

            except KeyboardInterrupt:
                pass

    def __str__(self) -> str:
        pass
