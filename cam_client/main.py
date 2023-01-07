import argparse

from traffic_counter import TrafficCounter


def CLI():
    # Define default values here to make documentation self-updating
    min_area_default = 200
    direction_default = ["H", "0.5"]
    num_count_default = 10
    video_width_default = 640
    video_params_default = ["mjpg", "avi"]
    starting_frame_default = 10

    parser = argparse.ArgumentParser(
        description="Finds the contours on a video file"
    )  # creates a parser object
    parser.add_argument(
        "-p",
        "--path",
        type=str,
        help="""A video filename or path.
    Works better with .avi files.
    If no path or name is provided, the camera will be used instead.""",
    )  # instead of using metavar='--path', just type '--path'. For some reason the metavar argument was causing
    # problems
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
        "-w",
        "--webcam",
        type=int,
        nargs="+",
        help="""Allows the user to specify which to use as the video source""",
    )
    parser.add_argument(
        "--rgb", action="store_true", help="Boolean flag to use rbg colors."
    )
    parser.add_argument(
        "-vo",
        "--video_out",
        type=str,
        default="",
        help="Provide a video filename to output",
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
        "-vp",
        "--video_params",
        type=str,
        default=video_params_default,
        nargs=2,
        help=f"Provide video codec and extension (in that order) for the output video. Example: `--video_params mjpg "
             f"avi`. Default values are {video_params_default[0]} {video_params_default[1]}",
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
        "--server", default="tcp://localhost:5555", help="img server address"
    )
    return parser.parse_args()


def make_video_params_dict(video_params):
    codec = video_params[0]
    extension = video_params[1]

    params_dict = {
        "codec": codec,
        "extension": extension,
    }
    return params_dict


def main(opts):
    """
    main
    """
    video_source = opts.path
    line_direction = opts.direction[0]
    line_position = float(opts.direction[1])
    video_width = opts.video_width
    min_area = int(opts.min_area)
    video_out = opts.video_out
    num_cnts = int(opts.numCount)
    video_params = make_video_params_dict(opts.video_params)
    starting_frame = opts.starting_frame
    img_server  = opts.server
    tc = TrafficCounter(
        video_source,
        line_direction,
        line_position,
        video_width,
        min_area,
        video_out,
        num_cnts,
        video_params,
        starting_frame,
        img_server
    )

    tc.main_loop()


if __name__ == "__main__":
    args = CLI()
    main(args)
