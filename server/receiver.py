from flask import render_template, Flask, Response, request
from flask_socketio import emit, SocketIO
from imagezmq import ImageHub


class Server:
    image_hub: ImageHub
    app: Flask
    socketio: SocketIO

    def __init__(self):
        self.running = True

    def run_server(self):
        app = self.app = Flask(__name__)
        self.socketio = socketio = SocketIO(app, async_mode=None)
        self.image_hub = ImageHub()

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
                # process_detections(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @app.post("/count")
        def count():
            data = request.get_json()
            # todo detect
            print(data)
            return "ok"

        @app.post("/sensors")
        def sensors():
            data = request.get_json()
            socketio.emit("sensors", data)
            return "ok"

        @socketio.on('connect')
        def connect():
            emit('connect', {'data': 'Connected to client!'})
            # global thread
            # with thread_lock:
            #     if thread is None:
            #         thread = socketio.start_background_task(test_sockets)

        @socketio.on('disconnect')
        def disconnect():
            print('Client disconnected!')

        app.run("localhost", 5000)


if __name__ == "__main__":
    server = Server()
    server.run_server()
