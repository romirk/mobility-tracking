from flask import render_template, Flask, Response, request
from flask_socketio import emit, SocketIO
from imagezmq import ImageHub


class Server:
    image_hub: ImageHub
    app: Flask
    socketio: SocketIO

    def __init__(self):
        self.running = True

    @staticmethod
    def create_server():
        app = Flask(__name__)
        socketio = SocketIO(app)

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
            return data

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

        return app, socketio

    def run(self):
        self.app, self.socketio = self.create_server()
        self.image_hub = ImageHub()
        self.app.run("localhost", 5000, threaded=False)


if __name__ == "__main__":
    server = Server()
    server.run()
