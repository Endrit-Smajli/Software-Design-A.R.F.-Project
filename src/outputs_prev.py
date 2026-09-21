from flask import Flask, Response   # HTTP endpoints, stream MJPEG frames.
import cv2                          # JPEG encoding and box drawing.
import json                         # Convert tracking dictionaries to JSON for ROS2 publishing.
import threading                    # Run ROS2 in a background thread while the Flask server runs in the main thread.

import rclpy                        
from rclpy.node import Node         # ROS2 node for publishing tracking data.
from std_msgs.msg import String     # Tracking data published as JSON string messages.

# Create Flask app.
app = Flask(__name__)

# -----------------------------
# ROS2 Publisher
# -----------------------------
class RosTrackPublisher(Node):
    
    # Initialize ROS2 node.
    def __init__(self):
        
        # Node name is 'oakd_tracks', which will be used in ROS2 to identify this node.
        super().__init__('oakd_tracks')
        
        # Create ROS2 publisher. 
        # - Topic: /tracked_objects
        # - Message type: String
        # - Queue size: 10.
        self.pub = self.create_publisher(String, 'tracked_objects', 10)
        
        # Store the latest tracking data to be published.
        self.latest_tracks = [] 
        
        # Create ROS2 timer to call timer_callback every 0.05 seconds (20 Hz).
        self.timer = self.create_timer(0.05, self.timer_callback)   

    # Publish tracking data at 20Hz on the /tracked_objects topic. 
    # The data is converted to JSON string format before publishing.
    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.latest_tracks)
        self.pub.publish(msg)

# Run ROS2 publisher in a background thread so that it doesn't block the Flask server.
def start_ros_publisher(publisher_holder):
    
    rclpy.init()                    # Initialize ROS2 system
    node = RosTrackPublisher()      # Create ROS2 node
    publisher_holder["node"] = node # Store node in a shared dictionary to update `latest_tracks` from the Flask server.
    rclpy.spin(node)                # Start ROS2 event loop
    
    # Clean up ROS2 resources when the thread is stopped.
    node.destroy_node()            
    rclpy.shutdown()


# -----------------------------
# Flask Server
# -----------------------------
class FlaskServer:
    
    # Constructor recieves  and stores the OAK-D device, YOLO drawer, 
    # tracker, and ROS2 publisher holder for later use.
    def __init__(self, device, yolo_drawer, tracker, ros_holder):
        self.device = device
        self.yolo_drawer = yolo_drawer
        self.tracker = tracker
        self.ros_holder = ros_holder

        # Register all Flask endpoints (routes) for streaming video and tracking data.
        self._register_routes()

    # -----------------------------
    # ROUTES - Define all HTTP endpoints for the Flask server.
    # -----------------------------
    def _register_routes(self):
        
        # Raw RGB frames
        @app.route("/rgb")
        def rgb_stream():
            return Response(self._gen_rgb(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        # YOLO detections
        @app.route("/yolo")
        def yolo_stream():
            return Response(self._gen_yolo(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        # Tracking IDs only
        @app.route("/track")
        def track_stream():
            return Response(self._gen_track(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

        # YOLO + tracking combined
        @app.route("/unified")
        def unified_stream():
            return Response(self._gen_unified(),
                            mimetype="multipart/x-mixed-replace; boundary=frame")

    # -----------------------------
    # JPEG ENCODER
    # -----------------------------
    
    # Encode a frame as JPEG for MJPEG streaming
    def _encode(self, frame):
        
        # Compress frame into JPEG
        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        # Skip frame if encoding fails.
        if not ret:
            return None
        
        # Return MJPEG chunk
        return (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )

    # -----------------------------
    # STREAM 1 — RAW RGB
    # Read raw frames -> encodes -> streams
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
    # Get queues + initialize detection list
    # -----------------------------
    def _gen_yolo(self):
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn
        detections = []

        # Get RGB frame
        while True:
            frame = q_rgb.get().getCvFrame()
            
            # Get YOLO detectins (non-blocking)
            in_nn = q_nn.tryGet()
            if in_nn is not None:
                detections = in_nn.detections

            # Draw YOLO boxes
            frame = self.yolo_drawer.draw(frame, detections)

            # Encode + stream
            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # STREAM 3 — TRACKING ONLY
    # -----------------------------
    def _gen_track(self):
        
        # Get queues
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn

        # Get frame + detections
        while True:
            frame = q_rgb.get().getCvFrame()
            in_nn = q_nn.tryGet()
            detections = in_nn.detections if in_nn is not None else []

            # Run tracking
            tracks = self.tracker.update(detections, frame.shape)

            # Update ROS2 publisher
            node = self.ros_holder.get("node")
            if node is not None:
                node.latest_tracks = tracks

            # Draw tracking boxes + IDs
            frame = self.tracker.draw_tracks(frame, tracks)

            # Encode + stream
            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # STREAM 4 — YOLO + TRACKING COMBINED
    # -----------------------------
    def _gen_unified(self):
        
        # Get queues
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn

        # Get RGB frame
        while True:
            frame = q_rgb.get().getCvFrame()

            # Get YOLO detections
            in_nn = q_nn.tryGet()
            detections = in_nn.detections if in_nn is not None else []

            # Run tracking
            tracks = self.tracker.update(detections, frame.shape)

            # Update ROS2
            node = self.ros_holder.get("node")
            if node is not None:
                node.latest_tracks = tracks

            # Draw YOLO boxes
            frame = self.yolo_drawer.draw(frame, detections)

            # Draw tracking IDs
            frame = self.tracker.draw_tracks(frame, tracks)

            # Encode + stream
            chunk = self._encode(frame)
            if chunk:
                yield chunk

    # -----------------------------
    # START Flask SERVER
    # -----------------------------
    def start(self):
        app.run(host="0.0.0.0", port=5000, threaded=True)
