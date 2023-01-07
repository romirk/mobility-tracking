import argparse
import multiprocessing
import queue
import socket

import imagezmq
import numpy as np
import pyrealsense2 as rs
from lane_tracker import IMG_HEIGHT, LaneTracker


class MobilityTracking:
    def __init__(self, args):
        self.name = socket.gethostname()
        self.n: int = args.n
        self.scan_x: int = args.scan_x if args.scan_x >= 0 else 0
        self.scan_y: int = args.scan_y if args.scan_y >= 0 else 0
        self.scan_w: int = args.scan_width
        self.scan_h: int = args.scan_height
        self.fps: int = args.frame_rate
        self.alpha_t: float = args.alpha
        self.direction: str = args.direction
        self.pipeline, self.pipeline_config = MobilityTracking._create_pipeline(
            640, 480, self.fps
        )  # TODO: use args

        self.server = args.server

        self.sender = imagezmq.ImageSender(connect_to=self.server)
        self.count = multiprocessing.Value("i", 0)

        if self.direction == "horizontal":
            self.lane_width = self.scan_w / self.n
        else:
            self.lane_width = self.scan_h / self.n

        self.lane_trackers: list[LaneTracker] = []
        self.processes: list[multiprocessing.Process] = []
        self.barrier = multiprocessing.Barrier(self.n)
        for i in range(self.n):
            config = LaneTracker.build_configuration(
                {"fps": self.fps, "alpha_t": self.alpha_t}
            )

            if self.direction == "horizontal":
                (
                    config.scan_x(self.scan_y + i * self.lane_width)
                    .scan_y(self.scan_y)
                    .scan_w(self.lane_width)
                    .scan_h(self.scan_h)
                )
            else:
                (
                    config.scan_x(self.scan_x)
                    .scan_y(self.scan_y + i * self.lane_width)
                    .scan_w(self.scan_w)
                    .scan_h(self.lane_width)
                )

            tracker = LaneTracker(
                config.build(), self.pipeline, self.count, self.barrier, i
            )
            self.lane_trackers.append(tracker)
            self.processes.append(multiprocessing.Process(target=tracker, daemon=True))

        # TODO multiprocessing

    @staticmethod
    def _create_pipeline(width, height, fps):
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print(f"{device} | {device_product_line}\n")

        # find RGB sensor
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                break
        else:
            print("Requires Depth camera with Color sensor")
            exit(0)

        # enable depth stream
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        # enable RGB stream
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        pipeline.start(config)
        return pipeline, config

    @staticmethod
    def add_args(parser=argparse.ArgumentParser(description=__doc__)):
        parser.add_argument("-v", "--version", action="version", version="%(prog)s 1.0")
        parser.add_argument(
            "-a",
            "--alpha",
            default=80,
            type=float,
            help="Alpha threshold for the convolution filter",
        )
        parser.add_argument(
            "-f", "--frame-rate", default=30, type=int, help="Frame rate (fps)"
        )
        parser.add_argument(
            "-H", "--height", default=10, type=int, help="Camera height (m)"
        )
        parser.add_argument(
            "-w", "--scan-width", default=400, type=int, help="Scan width (px)"
        )
        parser.add_argument(
            "-l", "--scan-height", default=IMG_HEIGHT, type=int, help="Scan height (px)"
        )
        parser.add_argument(
            "-x", "--scan-x", default=-1, type=int, help="Scan x position (px)"
        )
        parser.add_argument(
            "-y", "--scan-y", default=-1, type=int, help="Scan y position (px)"
        )
        parser.add_argument(
            "-T",
            "--max-T",
            default=4.948422901931615,
            type=float,
            help="Max time period (s)",
        )
        parser.add_argument(
            "-t",
            "--min-T",
            default=0.8247371503219356,
            type=float,
            help="Min time period (s)",
        )
        parser.add_argument("-b", "--buffer-size", type=int, help="Buffer size")
        parser.add_argument(
            "-r",
            "--rate-of-influence",
            default=0.01,
            type=float,
            help="Rate of influence",
        )
        parser.add_argument("-n", default=1, type=int, help="Number of lanes")
        parser.add_argument("-d", "--direction", default="horizontal", help="Direction")
        parser.add_argument(
            "--server", default="tcp://localhost:5555", help="yolo server address"
        )
        return parser

    def run(self):
        # start lane trackers
        f = 0
        try:
            for process in self.processes:
                process.start()
            while True:
                frames = np.asanyarray(
                    self.pipeline.wait_for_frames().get_color_frame().get_data()
                )
                self.sender.send_image(self.name, frames)
                s = "\r"
                for tracker in self.lane_trackers:
                    try:
                        tracker.q.put(frames, timeout=1)
                    except queue.Full:
                        pass
                    # with tracker.b.get_lock():
                    #     s += "".join(str(e) for e in tracker.b) + " "
                with self.count.get_lock():
                    print(s + f"{self.count.value} {f}", end="")
                f += 1
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"ERR: {e}")
        finally:
            print("\nStopping lane trackers...")
            for tracker in self.lane_trackers:
                try:
                    tracker.q.put(None, timeout=1)
                except queue.Full:
                    pass
            self.pipeline.stop()

            # for process in self.processes:
            #     process.join()
            # self.barrier.wait()
            for process in self.processes:
                process.terminate()

            print(f"Count: {self.count.value}")


if __name__ == "__main__":
    parser = MobilityTracking.add_args()
    args = parser.parse_args()
    MobilityTracking(args).run()
