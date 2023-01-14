from argparse import Namespace
from multiprocessing import Value
from multiprocessing.shared_memory import SharedMemory
from threading import Thread

import cv2
import numpy as np
import yolov7_package
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit

from server.frame_server import FrameServer
from server.utils import IMG_SIZE, decode64, encode64

WHITELIST = {1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


class HttpServer:
    app: Flask
    socketio: SocketIO

    def __init__(self):
        self.__running = Value("b", True)
        self.__last_active = {}

        self.__counts = Namespace(car=0, person=0, truck=0, bus=0, motorcycle=0, bicycle=0)

        self.fs = FrameServer(self.__running)
        self.__mem = SharedMemory(name=self.fs.mem.name)
        self.__last_frame: np.ndarray = np.ndarray(IMG_SIZE, dtype=np.uint8, buffer=self.__mem.buf)

        self.fs_thread = Thread(target=self.fs.run, daemon=True)
        self.yolo = yolov7_package.Yolov7Detector()
        print("Server initialized")

    @property
    def counts(self):
        return self.__counts

    def kill(self):
        self.__running.value = False

    def __gen_frames(self):
        while self.__running.value:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + \
                cv2.imencode(".jpg", np.asarray(self.__last_frame).reshape(IMG_SIZE))[1].tobytes() + b"\r\n"

    def detect(self, img):
        cl, bx, sc = self.yolo.detect(img)
        cl: list = cl[0]  # ugh

        d = {"frame": encode64(img)}

        for i in range(len(cl)):
            if cl[i] not in WHITELIST or sc[i] < 0.7:
                continue
            if cl[i] not in d:
                d[cl[i]] = {
                    "name": WHITELIST[cl[i]],
                    "boxes": [bx[i]],
                    "scores": [sc[i]],
                    "count": 1
                }
            else:
                d[cl[i]]["boxes"].append(bx[i])
                d[cl[i]]["scores"].append(sc[i])
                d[cl[i]]["count"] += 1

        s = ""
        for c in WHITELIST:
            if c in d:
                name, count = d[c]['name'], d[c]['count']
                s += f"{count} {name}{'' if count == 1 else 's'} "
        d["str"] = s if len(s) else "no detections."
        return d

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
            x, y, w, h = decode64(data["rect"])
            direction = data["direction"]
            cropped = frame[y:y + h, x:x + w]
            result = self.detect(cropped)
            print(result["str"], "in", direction)
            socketio.emit('detect', {
                "count": data["count"],
                "data": result,
                "T": data["T"],
            })
            cv2.imwrite(f"output/{data['count']}.jpg", cropped)
            return "ok"

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
