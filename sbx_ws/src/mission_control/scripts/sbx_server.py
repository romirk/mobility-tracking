from __future__ import annotations

from multiprocessing import Event

import cv2
import numpy as np
import yolov7_package
from flask import Flask, Response, render_template
from flask_socketio import SocketIO, emit
from sensor_msgs.msg import CompressedImage
from sensorbox.msg import AQI
from utils import IMG_SIZE
from vision_msgs.msg import BoundingBox2DArray
from yolov7_package.model_utils import coco_names
import message_filters
import rospy


WHITELIST = {1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}
CONFIDENCE_THRESHOLD = 0.3


class SbxServer:
    app: Flask
    socketio: SocketIO

    def __init__(self):
        rospy.init_node("backend")
        self.stopped = Event()

        self.__last_active = 0
        self.__last_frame = np.zeros(IMG_SIZE, dtype=np.uint8)
        self.__aqi = AQI()

        self.__cache = message_filters.Cache(10)

        self.__counts = {
            "bicycle": {"total": 0},
            "car": {"total": 0},
            "motorcycle": {"total": 0},
            "bus": {"total": 0},
            "truck": {"total": 0},
        }
        self.__total = 0

        self.yolo = yolov7_package.Yolov7Detector(traced=True)

        self.img_sub = rospy.Subscriber(
            "/camera/color/image_raw/compressed", CompressedImage, self.camera_callback
        )
        self.bb_sub = rospy.Subscriber(
            "/sbx/boxes", BoundingBox2DArray, self.bb_callback
        )
        self.aqi_sub = rospy.Subscriber("/sbx/aqi", AQI, self.aqi_callback)


    @property
    def counts(self):
        return self.__counts

    @property
    def running(self):
        return not self.stopped.is_set()

    @property
    def total(self):
        return self.__total

    def camera_callback(self, img: CompressedImage):
        if not self.running:
            return
        self.__last_active = img.header.stamp.to_sec()
        self.__last_frame = cv2.imdecode(
            np.frombuffer(img.data, np.uint8), cv2.IMREAD_COLOR
        )

    def bb_callback(self, bb: BoundingBox2DArray):
        if not self.running:
            return
        classes, scores, boxes = self.detect(self.__last_frame)

        cx, cy = IMG_SIZE[0] // 2, IMG_SIZE[1] // 2
        cl, sc, bx = self.correlate(classes, boxes, scores, cx, cy)
        if cl:
            self.__counts[cl]["total"] += 1
            self.__total += 1

        self.socketio.emit(
            "bb",
            {
                "data": {
                    "classes": classes,
                    "scores": scores,
                    "boxes": boxes,
                    "correlated": {
                        "class": cl,
                        "score": sc,
                        "box": bx,
                    },
                }
            },
        )

    def aqi_callback(self, aqi: AQI):
        if not self.running:
            return
        self.__aqi = aqi
        self.socketio.emit("aqi", {"data": self.__aqi})

    def stop(self):
        self.stopped.set()

    def __gen_frames(self):
        while self.running:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + cv2.imencode(
                ".jpg", np.asarray(self.__last_frame).reshape(IMG_SIZE)
            )[1].tobytes() + b"\r\n"

    def detect(self, img: np.ndarray) -> tuple:
        return tuple(e for e in zip(self.yolo.detect(img)) if e[0] in WHITELIST)

    def correlate(
        self, classes, boxes, scores, cx, cy
    ) -> tuple[str, float, list[int]] | tuple[None, None, None]:
        min_dist = IMG_SIZE[0] + IMG_SIZE[1]
        min_idx = -1
        for i in range(len(classes)):
            if classes[i] not in WHITELIST or scores[i] < CONFIDENCE_THRESHOLD:
                continue

            b = boxes[i]
            bx, by = b[0] + b[2] // 2, b[1] + b[3] // 2
            diff = [bx - cx, by - cy]
            dist = np.linalg.norm(diff)
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        if min_idx == -1:
            return None, None, None

        min_cl = classes[min_idx]
        min_sc = scores[min_idx]
        min_bx = boxes[min_idx]

        return coco_names[min_cl], min_sc, min_bx

    def create_server(self):
        app = Flask(__name__)
        socketio = SocketIO(app)

        @app.route("/")
        def index():
            return render_template("index.html")

        @app.route("/exit")
        def stop():
            self.stop()
            return "Server stopped"

        @app.route("/live")
        def video_feed():
            return Response(
                self.__gen_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @socketio.on("connect")
        def connect():
            emit("info", {"data": "Connected to client!"})

        @socketio.on("disconnect")
        def disconnect():
            print("Client disconnected!")

        self.app, self.socketio = app, socketio

    def run(self):
        self.create_server()
        self.app.run("0.0.0.0", 5000)
        print("[INFO] stopping server")
