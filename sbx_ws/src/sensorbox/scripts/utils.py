import argparse
import codecs
import pickle
import sys
from time import sleep


def encode64(data):
    return codecs.encode(pickle.dumps(data), "base64").decode()


def decode64(data: str):
    return pickle.loads(codecs.decode(data.encode(), "base64"))


def countdown(secs):
    secs = int(secs)
    while secs:
        print(secs, end=" ")
        sleep(1)
        secs -= 1
    print("0")


def parse_args_counter(
    parser=argparse.ArgumentParser(
        description="traffic counter",
    )
):
    min_area_default = 200
    direction_default = ["H", "0.5"]
    num_count_default = 10
    video_width_default = 640
    video_height_default = 480
    starting_frame_default = 10

    parser.add_argument(
        "-a",
        "--min_area",
        type=int,
        help=f"The minimum area (in pixels) to draw a bounding box (default is {min_area_default})",
        default=min_area_default,
    )
    parser.add_argument(
        "-d",
        "--direction",
        type=str,
        default=direction_default,
        nargs=2,
        help=f"""A character: H or V
    representing the orientation of the count line. H is horizontal, V is vertical.
    If not provided, the default is {direction_default[0]},{direction_default[1]}. The second parameter
    is a float number from 0 to 1 indicating the place at which the
    line should be drawn.""",
    )
    parser.add_argument(
        "-n",
        "--num_contours",
        type=int,
        default=num_count_default,
        help=f"""The number of contours to be detected by the program (default is {num_count_default}).""",
    )
    parser.add_argument(
        "--rgb", action="store_true", help="Boolean flag to use rbg colors."
    )
    parser.add_argument(
        "-vw",
        "--video_width",
        type=int,
        default=video_width_default,
        help=f"Videos will be resized to this width (default is {video_width_default}). Height will be computed "
        f"automatically to preserve aspect ratio",
    )
    parser.add_argument(
        "-vh",
        "--video_height",
        type=int,
        default=video_height_default,
        help=f"Videos will be resized to this height (default is {video_width_default}). Width will be computed "
        f"automatically to preserve aspect ratio",
    )

    parser.add_argument(
        "-sf",
        "--starting_frame",
        type=int,
        default=starting_frame_default,
        help=f"Select the starting frame for video analysis (default is {starting_frame_default}). All frames before "
        f"that will still be used for the background average",
    )
    parser.add_argument(
        "-r",
        "--record",
        action="store_true",
        help="Boolean flag to record the video.",
    )
    parser.add_argument(
        "--debug", action="store_true", help="Boolean flag to show debug windows."
    )
    parser.add_argument(
        "-f",
        "--fps",
        type=int,
        default=30,
        help="Frames per second for the video (default is 30).",
    )
    return parser


def parse_args_sensorbox(
    parser=argparse.ArgumentParser(
        description="sensorbox",
    )
):
    baud_default = 115200
    server_default = "localhost"
    port_default = "COM4"

    parser.add_argument(
        "-p",
        "--port",
        type=str,
        help=f"Serial port (default is {port_default})",
        default=port_default,
    )
    parser.add_argument(
        "-s",
        "--server",
        type=str,
        default=server_default,
        help=f"Server address (default is {server_default})",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=baud_default,
        help=f"Serial baud rate (default is {baud_default})",
    )
    parser.add_argument(
        "-x", "--parallel", action="store_true", help="parallel processing"
    )
    parser.add_argument("-v", "--visual", action="store_true", help="visualize")
    return parser


def parse_args(
    parser=argparse.ArgumentParser(
        description="sensorbox",
    )
):
    parser = parse_args_counter(parser)
    parser = parse_args_sensorbox(parser)
    return parser.parse_args()
