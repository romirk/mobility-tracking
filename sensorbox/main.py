from sensorbox.traffic_counter import TrafficCounter
from sensorbox.utils import parse_args_counter


def main(opts):
    """
    main
    """
    # video_source = opts.path
    line_direction = opts.direction[0]
    line_position = float(opts.direction[1])
    video_width = opts.video_width
    min_area = int(opts.min_area)
    num_cnts = int(opts.numCount)
    starting_frame = opts.starting_frame
    img_server = opts.server
    arduino_port = opts.port
    tc = TrafficCounter(
        line_direction,
        line_position,
        video_width,
        min_area,
        num_cnts,
        starting_frame,
        img_server,
    )

    tc.main_loop()


if __name__ == "__main__":
    args = parse_args_counter()
    main(args)
