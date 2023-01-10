import ctypes
import datetime
from multiprocessing import Value, Process, Array

import cv2
import numpy as np
from flask import render_template, Flask, Response, request
from flask_socketio import emit, SocketIO
from imagezmq import ImageHub


class Server:
    image_hub: ImageHub
    app: Flask
    socketio: SocketIO

    __SIZE = (640, 480)

    def __init__(self):
        self.__running = Value("b", True)
        self.__last_active = {}

        self.__last_frame_buffer = Array(ctypes.c_uint8, Server.__SIZE[0] * Server.__SIZE[1])
        self.__last_frame = np.ndarray(Server.__SIZE, dtype=np.uint8, buffer=self.__last_frame_buffer.get_obj())

        print(self.__last_frame)

        self.receiver = Process(target=self.receive_frames, daemon=True)
        print("Server initialized")

    def kill(self):
        self.__running.value = False

    def __gen_frames(self):
        while self.__running.value:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + \
                cv2.imencode(".jpg", self.__last_frame).tobytes() + b"\r\n"

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

    def receive_frames(self):
        frame: np.ndarray
        self.image_hub = ImageHub()
        while self.__running.value:
            (hostname, frame) = self.image_hub.recv_image()
            self.image_hub.send_reply(b"OK")

            if hostname not in self.__last_active:
                print(f"[INFO] receiving data from {hostname}...")
            self.__last_active[hostname] = datetime.now()

            np.copyto(self.__last_frame, frame)
        print("[INFO] stopping frame receiver")

    def run(self):
        self.create_server()
        self.receiver.start()
        self.app.run("localhost", 5000, threaded=False)
        print("[INFO] stopping server")


if __name__ == "__main__":
    server = Server()
    server.run()
