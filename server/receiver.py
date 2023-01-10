import ctypes
import datetime
from multiprocessing import Value, Process, Array

import cv2
import numpy as np
from flask import render_template, Flask, Response, request
from flask_socketio import emit, SocketIO
from imagezmq import ImageHub


def receive_frames(running, last: Array):
    frame: np.ndarray
    image_hub = ImageHub()
    last_active = {}
    while running.value:
        (hostname, frame) = image_hub.recv_image()
        image_hub.send_reply(b"OK")

        if hostname not in last_active:
            print(f"[INFO] receiving data from {hostname}...")
        last_active[hostname] = datetime.datetime.now()

        frame = frame.flatten()
        # print(frame)
        with last.get_lock():
            # print(f"last: {last}")
            for i in range(len(last)):
                last[i] = frame[i]

    print("[INFO] stopping frame receiver")


class Server:
    image_hub: ImageHub
    app: Flask
    socketio: SocketIO

    __SIZE = (480, 640, 3)

    def __init__(self):
        self.__running = Value("b", True)
        self.__last_active = {}

        self.__last_frame: Array = Array(ctypes.c_uint8, Server.__SIZE[0] * Server.__SIZE[1] * 3)

        print(self.__last_frame)

        self.receiver = Process(target=receive_frames, args=(self.__running, self.__last_frame,), daemon=True)
        print("Server initialized")

    def kill(self):
        self.__running.value = False

    def __gen_frames(self):
        while self.__running.value:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + \
                cv2.imencode(".jpg", np.asarray(self.__last_frame).reshape(Server.__SIZE))[1].tobytes() + b"\r\n"

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

        @app.post("/count")
        def count():
            data = request.get_json()
            # todo detect
            print(data)
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
        self.receiver.start()
        self.app.run("localhost", 5000)
        print("[INFO] stopping server")


if __name__ == "__main__":
    server = Server()
    server.run()
