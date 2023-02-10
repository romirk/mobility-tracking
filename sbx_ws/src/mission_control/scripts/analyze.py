import cv2
import numpy as np
from yolov7_package import Yolov7Detector
import os
import torch.cuda

dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
print(dir)

if not os.path.exists(dir):
    os.makedirs(dir)
    exit(0)

yolo = Yolov7Detector(traced=True)
imgs = [
    cv2.imread(file)
    for file in os.listdir(dir)
    # if os.path.isfile(file) and file.endswith(".png")
]
print(len(imgs))
try:
    res = np.asanyarray(yolo.detect(imgs))
except Exception as e:
    res = np.asanyarray(yolo.detect(img) for img in imgs)

print(res.shape)
