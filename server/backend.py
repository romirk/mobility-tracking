#!./env/bin/python
from http_server import HttpServer

if __name__ == "__main__":
    server = HttpServer()
    server.run()
