#!/usr/bin/env python3

import http.server
import socketserver
import os

PORT = 8000
DIRECTORY = os.path.dirname(os.path.realpath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)


with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("serving at port", PORT)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
