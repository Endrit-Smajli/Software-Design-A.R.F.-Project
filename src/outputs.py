from flask import Flask, Response
import cv2
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

app = Flask(__name__)

# -----------------------------
# ROS2 Publisher
# -----------------------------
class RosTrackPublisher(Node):
    def __init__(self):
        super().__init__('oakd_tracks')
        self.pub = self.create_publisher(String, 'tracked_objects', 10)
        self.latest_tracks = []
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.latest_tracks)
        self.pub.publish(msg)


def start_ros_publisher(publisher_holder):
    rclpy.init()
    node = RosTrackPublisher()
    publisher_holder["node"] = node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# -----------------------------
# Flask Server
# -----------------------------
class FlaskServer:
    def __init__(self, device, yolo_drawer, tracker, ros_holder):
        self.device = device
        self.yolo_drawer = yolo_drawer
        self.tracker = tracker
        self.ros_holder = ros_holder

        self._register_routes()

    # -----------------------------
    # ROUTES
    # -----------------------------
    def _register_routes(self):

        @app.route("/rgb")
        def rgb_stream():
            return Response(self._gen_rgb(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/yolo")
        def yolo_stream():
            return Response(self._gen_yolo(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/track")
        def track_stream():
            return Response(self._gen_track(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/unified")
        def unified_stream():
            return Response(self._gen_unified(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

    # -----------------------------
    # JPEG ENCODER
    # -----------------------------
    def _encode(self, frame):
        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            return None
        return (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )

    # -----------------------------
    # STREAM 1 — RAW RGB
    # -----------------------------
    def _gen_rgb(self):
        q_rgb = self.device.q_rgb
        while True:
            frame = q_rgb.get().getCvFrame()
            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # STREAM 2 — YOLO ONLY
    # -----------------------------
    def _gen_yolo(self):
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn
        detections = []

        while True:
            frame = q_rgb.get().getCvFrame()
            in_nn = q_nn.tryGet()
            if in_nn is not None:
                detections = in_nn.detections

            frame = self.yolo_drawer.draw(frame, detections)

            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # STREAM 3 — TRACKING ONLY
    # -----------------------------
    def _gen_track(self):
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn

        while True:
            frame = q_rgb.get().getCvFrame()
            in_nn = q_nn.tryGet()
            detections = in_nn.detections if in_nn is not None else []

            tracks = self.tracker.update(detections, frame.shape)

            # Update ROS2 publisher
            node = self.ros_holder.get("node")
            if node is not None:
                node.latest_tracks = tracks

            frame = self.tracker.draw_tracks(frame, tracks)

            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # STREAM 4 — YOLO + TRACKING COMBINED
    # -----------------------------
    def _gen_unified(self):
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn

        while True:
            frame = q_rgb.get().getCvFrame()

            # YOLO detections
            in_nn = q_nn.tryGet()
            detections = in_nn.detections if in_nn is not None else []

            # SORT tracking
            tracks = self.tracker.update(detections, frame.shape)

            # ROS2 sync
            node = self.ros_holder.get("node")
            if node is not None:
                node.latest_tracks = tracks

            # Draw YOLO boxes
            frame = self.yolo_drawer.draw(frame, detections)

            # Draw tracking IDs
            frame = self.tracker.draw_tracks(frame, tracks)

            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # START SERVER
    # -----------------------------
    def start(self):
        app.run(host="0.0.0.0", port=5000, threaded=True)