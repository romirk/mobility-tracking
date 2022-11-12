import argparse
from datetime import datetime
from queue import Full
from threading import Thread
from uuid import uuid4

import cv2
import imagezmq
from detect import PROCESSED_Q, RAW_IMG_Q, detect
from flask import Flask, render_template
from flask.wrappers import Response
from utils.plots import plot_one_box

last_active = {}


imageHub = imagezmq.ImageHub()
app = Flask(__name__)


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights",
        nargs="+",
        type=str,
        default="../ignore/yolov7.pt",
        help="model.pt path(s)",
    )
    parser.add_argument(
        "--source", type=str, default="inference/images", help="source"
    )  # file/folder, 0 for webcam
    parser.add_argument(
        "--img-size", type=int, default=640, help="inference size (pixels)"
    )
    parser.add_argument(
        "--conf-thres", type=float, default=0.25, help="object confidence threshold"
    )
    parser.add_argument(
        "--iou-thres", type=float, default=0.45, help="IOU threshold for NMS"
    )
    parser.add_argument(
        "--device", default="", help="cuda device, i.e. 0 or 0,1,2,3 or cpu"
    )
    parser.add_argument("--view-img", action="store_true", help="display results")
    parser.add_argument("--save-txt", action="store_true", help="save results to *.txt")
    parser.add_argument(
        "--save-conf", action="store_true", help="save confidences in --save-txt labels"
    )
    parser.add_argument(
        "--classes",
        nargs="+",
        type=int,
        help="filter by class: --class 0, or --class 0 2 3",
    )
    parser.add_argument(
        "--agnostic-nms", action="store_true", help="class-agnostic NMS"
    )
    parser.add_argument("--augment", action="store_true", help="augmented inference")
    parser.add_argument("--update", action="store_true", help="update all models")
    parser.add_argument(
        "--project", default="runs/detect", help="save results to project/name"
    )
    parser.add_argument("--name", default="exp", help="save results to project/name")
    parser.add_argument(
        "--exist-ok",
        action="store_true",
        help="existing project/name ok, do not increment",
    )
    parser.add_argument("--no-trace", action="store_true", help="don`t trace model")
    opt = parser.parse_args()
    opt.nosave = True
    return opt


def receive_frames():
    while True:
        (hostname, frame) = imageHub.recv_image()
        # print(f"Received frame from {hostname}")
        imageHub.send_reply(b"OK")

        if hostname not in last_active:
            print(f"[INFO] receiving data from {hostname}...")
        last_active[hostname] = datetime.now()

        entry = {
            "uuid": str(uuid4()),
            "frame": frame,
            "source": hostname,
            "timestamp": last_active[hostname],
        }
        try:
            RAW_IMG_Q.put_nowait(entry)
        except Full:
            # print("[INFO] dropping frame from queue")
            pass


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/live")
def video_feed():
    return Response(
        process_detections(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def process_detections():
    while True:
        processed: dict = PROCESSED_Q.get(block=True)
        frame = processed["frame"]
        dets = processed["detections"]
        for cl, data in dets.items():
            for box, conf in zip(data["boxes"], data["confidences"]):
                plot_one_box(
                    box,
                    frame,
                    label=f"{cl}: {conf:.2f}",
                    color=data["color"],
                    line_thickness=3,
                )
        encoded = cv2.imencode(".jpg", frame)[1]
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + encoded.tobytes() + b"\r\n"


if __name__ == "__main__":
    opt = parse_arguments()
    proc_thread = Thread(target=receive_frames)
    proc_thread.start()
    detect_thread = Thread(target=detect, args=(opt,))
    detect_thread.daemon = True
    detect_thread.start()
    app.run("0.0.0.0", 8080)
