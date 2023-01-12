from __future__ import annotations

from celery import Celery, Task
from yolov7_package import Yolov7Detector

from utils import encode64

app = Celery('broker', backend='rpc://', broker='pyamqp://')

WHITELIST = {1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


class _DetectTask(Task):
    name = "Detect"
    description = "Real-time object detection via YOLOv7"
    public = True

    def __init__(self):
        self.det = Yolov7Detector(traced=False)

    def run(self, img):



app.register_task(_DetectTask())

detect = app.tasks[_DetectTask.name]
