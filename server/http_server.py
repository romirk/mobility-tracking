from multiprocessing import Value
from multiprocessing.shared_memory import SharedMemory
from threading import Thread
from typing import Tuple

import cv2
import numpy as np
import yolov7_package
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
from yolov7_package.model_utils import coco_names

# from server.frame_server import FrameServer
# from server.utils import IMG_SIZE, decode64
from frame_server import FrameServer
from utils import IMG_SIZE, decode64, expand_bounding_box

WHITELIST = {1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}
CONFIDENCE_THRESHOLD = 0.3


class HttpServer:
    app: Flask
    socketio: SocketIO

    def __init__(self):
        self.__running = Value("b", True)
        self.__last_active = {}

        self.__counts = {
            "bicycle": {},
            "car": {},
            "motorcycle": {},
            "bus": {},
            "truck": {},
        }
        self.__total = 0

        self.fs = FrameServer(self.__running)
        self.__mem = SharedMemory(name=self.fs.mem.name)
        self.__last_frame: np.ndarray = np.ndarray(IMG_SIZE, dtype=np.uint8, buffer=self.__mem.buf)

        self.fs_thread = Thread(target=self.fs.run, daemon=True)
        self.yolo = yolov7_package.Yolov7Detector(traced=True)
        print("Server initialized")

    @property
    def counts(self):
        return self.__counts

    @property
    def running(self):
        return self.__running.value

    @property
    def total(self):
        return self.__total

    def kill(self):
        self.__running.value = False

    def __gen_frames(self):
        while self.__running.value:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + \
                cv2.imencode(".jpg", np.asarray(self.__last_frame).reshape(IMG_SIZE))[1].tobytes() + b"\r\n"

    def detect(self, img, box) -> Tuple[str, float, Tuple[int, int, int, int]]:
        cx, cy = box[0] + box[2] // 2, box[1] + box[3] // 2
        classes, boxes, scores = self.yolo.detect(img)
        classes, boxes, scores = classes[0], boxes[0], scores[0]  # only 1 image

        min_dist = img.shape[0] + img.shape[1]
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
            self.__running.value = False

        @app.route("/live")
        def video_feed():
            return Response(
                self.__gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @app.post("/detect")
        def count():
            data = request.get_json()
            frame = decode64(data["frame"])
            box = x, y, w, h = decode64(data["rect"])
            direction = data["direction"]
            cls, score, box = self.detect(frame, box)

            x, y, w, h = expand_bounding_box(x, y, w, h, frame)

            cropped = frame[y:y + h, x:x + w]
            cv2.imwrite(f"server/static/{data['count']}.jpg", cropped)
            if cls is None:
                print("No object detected")
                return {"count": 0}

            if direction not in self.counts[cls]:
                self.counts[cls][direction] = 1
                self.counts[cls]["total"] = 1
            else:
                self.counts[cls][direction] += 1
                self.counts[cls]["total"] += 1
            self.__total += 1

            socketio.emit('detect', {
                "counts": self.counts,
                "T": data["T"],
                "total": self.total,
                "cam_count": data["count"],
            })
            print(f"Detected {cls} moving {direction} with confidence {score} at {box}")

            return {"count": self.total}

        @app.post("/sensors")
        def sensors():
            data = request.get_json()
            socketio.emit("sensors", data)
            return "ok"

        @socketio.on('connect')
        def connect():
            emit('info', {'data': 'Connected to client!'})

        @socketio.on('disconnect')
        def disconnect():
            print('Client disconnected!')

        self.app, self.socketio = app, socketio

    def run(self):
        self.create_server()
        self.fs_thread.start()
        self.app.run("0.0.0.0", 5000)
        print("[INFO] stopping server")
