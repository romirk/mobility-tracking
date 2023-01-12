import codecs
import datetime
from multiprocessing import Value, Process
from multiprocessing.shared_memory import SharedMemory

import cv2
import numpy as np
from flask import render_template, Flask, Response, request
from flask_socketio import emit, SocketIO
from imagezmq import ImageHub

from broker import detect
from utils import decode64


def receive_frames(running, loc: str):
    frame: np.ndarray
    mem = SharedMemory(name=loc)
    last = np.ndarray(Server.IMG_SIZE, dtype=np.uint8, buffer=mem.buf)
    image_hub = ImageHub()
    last_active = {}
    while running.value:
        (hostname, frame) = image_hub.recv_image()
        image_hub.send_reply(b"OK")

        if hostname not in last_active:
            print(f"[INFO] receiving data from {hostname}...")
        last_active[hostname] = datetime.datetime.now()

        np.copyto(last, frame)

    print("[INFO] stopping frame receiver")


class Server:
    image_hub: ImageHub
    app: Flask
    socketio: SocketIO

    IMG_SIZE = (480, 640, 3)

    def __init__(self):
        self.__running = Value("b", True)
        self.__last_active = {}

        self.__mem = SharedMemory(create=True, size=Server.IMG_SIZE[0] * Server.IMG_SIZE[1] * Server.IMG_SIZE[2])
        self.__last_frame: np.ndarray = np.ndarray(Server.IMG_SIZE, dtype=np.uint8, buffer=self.__mem.buf)

        self.receiver = Process(target=receive_frames, args=(self.__running, self.__mem.name), daemon=True)
        print("Server initialized")

    def kill(self):
        self.__running.value = False

    def __gen_frames(self):
        while self.__running.value:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + \
                cv2.imencode(".jpg", np.asarray(self.__last_frame).reshape(Server.IMG_SIZE))[1].tobytes() + b"\r\n"

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
        self.app.run("0.0.0.0", 5000)
        print("[INFO] stopping server")


if __name__ == "__main__":
    server = Server()
    server.run()
