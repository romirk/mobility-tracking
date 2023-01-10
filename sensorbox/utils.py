import argparse


def parse_args_counter():
    min_area_default = 200
    direction_default = ["H", "0.5"]
    num_count_default = 10
    video_width_default = 640
    video_params_default = ["mjpg", "avi"]
    starting_frame_default = 10
    server_default = "tcp://localhost:5555"

    parser = argparse.ArgumentParser(
        description="Finds the contours on a video file"
    )
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
        "--numCount",
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
        "-sf",
        "--starting_frame",
        type=int,
        default=starting_frame_default,
        help=f"Select the starting frame for video analysis (default is {starting_frame_default}). All frames before "
             f"that will still be used for the background average",
    )
    parser.add_argument(
        "--server", default=server_default, help="img server address"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="arduino port")
    return parser.parse_args()


def make_video_params_dict(video_params):
    codec = video_params[0]
    extension = video_params[1]

    params_dict = {
        "codec": codec,
        "extension": extension,
    }
    return params_dict
