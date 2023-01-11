import codecs
import pickle


def encode64(data):
    return codecs.encode(pickle.dumps(data), "base64").decode()


def decode64(data: str):
    return pickle.loads(codecs.decode(data.encode(), "base64"))
