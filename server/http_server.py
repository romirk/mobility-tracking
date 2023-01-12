import codecs
from argparse import Namespace
from multiprocessing import Value
from multiprocessing.shared_memory import SharedMemory
from threading import Thread

import cv2
import numpy as np
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit

from server.broker import detect
from server.frame_server import FrameServer
from server.utils import IMG_SIZE, decode64


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
            cropped = frame[y:y + h, x:x + w]
            result = detect(cropped)
            print(result["str"])
            socketio.emit('detect', {
                "count": data["count"],
                "frame": codecs.encode(cv2.imencode(".jpg", cropped)[1], "base64").decode(),
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
