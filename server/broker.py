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
        cl, bx, sc = self.det.detect(img)
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


app.register_task(_DetectTask())

detect = app.tasks[_DetectTask.name]
