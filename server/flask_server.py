import argparse
from datetime import datetime
from multiprocessing import Process
from queue import Full
from threading import Thread, Lock
from uuid import uuid4

import cv2
import imagezmq
import numpy as np
from flask import Flask, render_template
from flask.wrappers import Response
from flask_socketio import SocketIO, emit

from detect import PROCESSED_Q, RAW_IMG_Q, detect
from utils.args import parse_arguments
from utils.plots import plot_one_box

last_active = {}

app = Flask(__name__)

imageHub = None
socketio = SocketIO(app, async_mode=None)
thread_lock = Lock()
thread = None
running = True


def update_count(new_count, frame, box):
    ...



last_entry = {
    "uuid": 0,
    "frame": np.random.random((640, 480)) * 255,
    "source": "test",
    "timestamp": 0,
}
last_frame = last_entry["frame"]


def receive_frames():
    global last_entry, last_frame
    while running:
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
        last_entry = entry
        last_frame = last_entry["frame"]
        try:
            RAW_IMG_Q.put_nowait(entry)
        except Full:
            # print("[INFO] dropping frame from queue")
            pass
    print("[INFO] stopping frame receiver")


@app.route("/")
def index():
    return render_template("index.html")

@app.route("/exit")
def stop():
    global running
    running = False


@app.route("/live")
def video_feed():
    return Response(
        process_detections(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def gen_frames():
    while running:
        encoded = cv2.imencode(".jpg", last_frame)[1]
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + encoded.tobytes() + b"\r\n"


@socketio.on('connect')
def connect():
    emit('connect', {'data': 'Connected to client!'})
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(test_sockets)


@socketio.on('disconnect')
def disconnect():
    print('Client disconnected!')


def test_sockets():
    counter = 0

    while True:
        counter = counter + 1
        socketio.emit('traffic_data', {'data': 'Car count', 'count': counter})
        print(counter)
        socketio.sleep(1)


def process_detections():
    global last_frame, running
    while running:
        processed: dict = PROCESSED_Q.get(block=True)
        print(processed)
        print(":")
        if processed is None:
            print("poison pill")
            break
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
        last_frame = frame
        encoded = cv2.imencode(".jpg", last_frame)[1]
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + encoded.tobytes() + b"\r\n"
    print("[INFO] stopping detection process")


def run_sever():
    global imageHub
    imageHub = imagezmq.ImageHub()
    app.run(host="0.0.0.0", port=5000)

if __name__ == "__main__":
    opt = parse_arguments()
    rec_thread = Thread(target=receive_frames)
    det_thread = Process(target=detect, args=(opt,))
    app_thread = Thread(target=run_sever)
    try:
        print("start")
        rec_thread.start()
        det_thread.start()
        app_thread.start()
        print("running")
        app_thread.join()
    except (KeyboardInterrupt, SystemExit):
        print("[INFO] exiting...")
        running = False
        print("[INFO] clearing queues...")
        with RAW_IMG_Q.mutex:
            RAW_IMG_Q.queue.clear()
        with PROCESSED_Q.mutex:
            PROCESSED_Q.queue.clear()
        PROCESSED_Q.put(None, block=True)
        RAW_IMG_Q.put(None, block=True)
        print("[INFO] main thread terminated")
        exit(0)
