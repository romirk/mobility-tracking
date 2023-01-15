from __future__ import annotations

import codecs
import pickle

IMG_SIZE = (480, 640, 3)
IMG_SIZE_RGB = (480, 640, 3)


def encode64(data: object):
    return codecs.encode(pickle.dumps(data), "base64").decode()


def decode64(data: str | bytes):
    return pickle.loads(codecs.decode(data.encode() if type(data) is str else data, "base64"))


def expand_bounding_box(x, y, w, h, frame):
    cx = x + w / 2
    cy = y + h / 2

    w *= 1.5
    h *= 1.5

    x = cx - w / 2
    y = cy - h / 2

    x, y, w, h = tuple(map(int, (x, y, w, h)))

    if x < 0:
        x = 0

    if y < 0:
        y = 0

    if x + w >= frame.shape[1]:
        w = frame.shape[1] - x

    if y + h >= frame.shape[0]:
        h = frame.shape[0] - y

    return x, y, w, h
