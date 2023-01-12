from __future__ import annotations

import codecs
import pickle

IMG_SIZE = (480, 640, 3)

def encode64(data: object):
    return codecs.encode(pickle.dumps(data), "base64").decode()


def decode64(data: str | bytes):
    return pickle.loads(codecs.decode(data.encode(), "base64"))
