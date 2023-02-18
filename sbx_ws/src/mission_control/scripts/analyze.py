from __future__ import annotations
import cv2
import numpy as np
import os
from det_executor import DetExecutor

dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
print(dir)

if not os.path.exists(dir):
    os.makedirs(dir)
    exit(0)

yolo = DetExecutor("yolov7-traced")
imgs = [
    cv2.imread(file)
    for file in os.listdir(dir)
    # if os.path.isfile(file) and file.endswith(".png")
]
print(len(imgs))
try:
    res = np.asanyarray(yolo.predict(imgs))
except Exception as e:
    res = np.asanyarray(yolo.predict(img) for img in imgs)

print(res.shape)
