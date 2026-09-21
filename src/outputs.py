from flask import Flask, Response, render_template_string   # HTTP endpoints, stream MJPEG frames.
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
    
    # Constructor recieves and stores the OAK-D device, YOLO drawer, 
    # tracker, and ROS2 publisher holder for later use.
    def __init__(self, device, yolo_drawer, tracker, ros_holder):
        
        self.device = device
        self.yolo_drawer = yolo_drawer
        self.tracker = tracker
        self.ros_holder = ros_holder

        # -------------------
        # Shared frame buffer
        # --------------------
            
        self.latest_frames = {
            "rgb": None,
            "yolo": None,
            "track": None,
            "unified": None
        }
        
        # Tell Flask clients when a new frame is available for streaming.
        self.frame_number = 0
        
        # Shared frame buffer protection
        self.frame_condition = threading.Condition()
        
        # Register all Flask endpoints (routes) for streaming video and tracking data.
        self._register_routes()

        self.processing_thread = threading.Thread(target = self._processing_loop, daemon = True)
        
        # Start ROS2 publisher in a background thread.
        self.processing_thread.start() 
    
    # --------------------------------------------------------
    # ROUTES - Define all HTTP endpoints for the Flask server.
    # --------------------------------------------------------
    def _register_routes(self):
        
        # Main page with links to all streams
        @app.route("/")
        @app.route("/dashboard")
        def dashboard():
            
            return render_template_string("""
<!DOCTYPE html>

<html>

<head>

    <title>Search and Rescue camera view (OAK-D Lite)</title>
    
    <style>
    
        * {
            box-sizing: border-box;
        }
        
        html, body {
            margin: 0;
            padding: 0;
            width: 100%;
            height: 100%;
        }
        
        body {
            background: #111;
            color: white;
            font-family: Arial, sans-serif;
            overflow: hidden;
        }
        
        /* Dashboard header */
        .header {
            height: 50px;
            display: flex;
            align-items: center;
            padding: 0 16px;
            background: #1b1b1b;
            font-size: 20px;
            font-weight: bold;
        }
        
        /* 2x2 camera layout */
        .grid {
            height: calc(100vh - 50px);
            
            display: grid;
            
            grid-template-columns: 1fr 1fr;
            grid-template-rows: 1fr 1fr;
            
            gap: 4px;
            
            padding: 4px;
        }
        
        /* Individual camera panel */
        .panel {
            position: relative;
            
            min-width: 0;
            min-height: 0;
            
            background: black;
            
            overflow: hidden;
        }
        
        /* Video */
        .panel img {
            width: 100%;
            height: 100%;
            
            object-fit: contain;
            
            display: block;
        }
        
        /* Stream title */
        .label {
            position: absolute;
            
            top: 8px;
            left: 8px;
            
            padding: 6px 10px;
            
            background: rgba(0, 0, 0, 0.7);

            border-radius: 4px;
            
            font-size: 14px;
            font-weight: bold;
        }
        
        </style>
        
</head>



<body>
    <div class = "header">
        Search and Rescue camera view (OAK-D Lite) Dashboard
    </div>
    
    <div class = "grid">
    
        <!-- Raw RGB -->
        <div class = "panel">
        
            <img
                src = "/rgb"
                alt = "RGB Camera Stream"
            >
            
            <div class = "label">
                RGB
            </div>
            
        </div>
        
        <!-- YOLO Detections -->
        <div class = "panel">
        
            <img
                src = "/yolo"
                alt = "YOLO Detections Stream"
            >
            
            <div class = "label">
                YOLO Detections
            </div>
            
        </div>
        
        <!-- ID Tracker -->
        <div class = "panel">
        
            <img
                src = "/track"
                alt = "ID Tracker Stream"
            >
            
            <div class = "label">
                ID Tracker
            </div>
        
        </div>
        
        <!-- YOLO + ID Tracker -->
        <div class = "panel">
        
            <img
                src = "/unified"
                alt = "YOLO + Tracking Stream"
            >
            
            <div class = "label">
                YOLO + ID Tracker
            </div>
            
        </div>
    
    </div>
    
</body> 

</html>
            """)
            
        # Raw RGB frames
        @app.route("/rgb")
        def rgb_stream():
            
            return Response(self._gen_shared("rgb"),
                            mimetype = "multipart/x-mixed-replace; boundary = frame")

        # YOLO detections
        @app.route("/yolo")
        def yolo_stream():
            
            return Response(self._gen_shared("yolo"),
                            mimetype = "multipart/x-mixed-replace; boundary = frame")

        # Tracking IDs only
        @app.route("/track")
        def track_stream():
            return Response(self._gen_shared("track"),
                            mimetype = "multipart/x-mixed-replace; boundary = frame")

        # YOLO + tracking combined
        @app.route("/unified")
        def unified_stream():
            return Response(self._gen_shared("unified"),
                            mimetype = "multipart/x-mixed-replace; boundary = frame")

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
    # SHARED PROCESSING LOOP
    # -----------------------------
    def _processing_loop(self):
        
        # Get OAK-D output queues
        q_rgb = self.device.q_rgb
        q_nn  = self.device.q_nn
        
        # Store the most recent YOLO detections
        detections = []
        
        while True:
            
            in_rgb = q_rgb.tryGet()     # Get RGB frame (non-blocking)
            
            if in_rgb is None:
                continue
            
            frame = in_rgb.getCvFrame() # Convert DepthAI frame to OpenCV format
            
            in_nn = q_nn.tryGet()       # Get YOLO detections (non-blocking)
            
            if in_nn is not None:
                detections = in_nn.detections
                
            #Create separate frame copies
            
            rgb_frame     = frame.copy()
            yolo_frame    = frame.copy()
            track_frame   = frame.copy()
            unified_frame = frame.copy()
            
            # Run tracker ONCE
            tracks = self.tracker.update(detections, frame.shape)
            
            # Update ROS2 publisher with latest tracks
            node = self.ros_holder.get("node")
            
            if node is not None:
                node.latest_tracks = tracks

            # YOLO view
            yolo_frame = self.yolo_drawer.draw(yolo_frame, detections)
            
            # Tracking view
            track_frame = self.tracker.draw_tracks(track_frame, tracks)
            
            # Unified view
            unified_frame = self.yolo_drawer.draw(unified_frame, detections)
            unified_frame = self.tracker.draw_tracks(unified_frame, tracks)
            
            # Encode all 4 frames as JPEG
            encoded_frames = {
                
                "rgb": self._encode(rgb_frame),
                "yolo": self._encode(yolo_frame),
                "track": self._encode(track_frame),
                "unified": self._encode(unified_frame) 
            }
            
            # Update shared frame buffer
            with self.frame_condition:
                
                self.latest_frames = encoded_frames
                
                self.frame_number += 1
                
                # Tell all Flask streams that a new frame is available.
                self.frame_condition.notify_all()
                
    # SHARED MJPEG GENERATOR
    def _gen_shared(self, stream_type):
        
        # Keep track of which frame the browser has already recieved
        last_frame_number = -1
        
        while True:
            
            with self.frame_condition:
                
                # Wait for a new frame to be available
                self.frame_condition.wait_for(lambda: self.frame_number != last_frame_number)
                
            
            # Get requested stream
            chunk = self.latest_frames.get(stream_type)

            # Remember this frame number
            last_frame_number = self.frame_number
            
            # Send JPEG frame to browser
            if chunk:
                yield chunk
            
            
    # # -----------------------------
    # # STREAM 1 — RAW RGB
    # # Read raw frames -> encodes -> streams
    # # -----------------------------
    # def _gen_rgb(self):
    #     q_rgb = self.device.q_rgb
    #     while True:
    #         frame = q_rgb.get().getCvFrame()
    #         chunk = self._encode(frame)
    #         if chunk:
    #             yield chunk

    # # -----------------------------
    # # STREAM 2 — YOLO ONLY
    # # Get queues + initialize detection list
    # # -----------------------------
    # def _gen_yolo(self):
    #     q_rgb = self.device.q_rgb
    #     q_nn  = self.device.q_nn
    #     detections = []

    #     # Get RGB frame
    #     while True:
    #         frame = q_rgb.get().getCvFrame()
            
    #         # Get YOLO detectins (non-blocking)
    #         in_nn = q_nn.tryGet()
    #         if in_nn is not None:
    #             detections = in_nn.detections

    #         # Draw YOLO boxes
    #         frame = self.yolo_drawer.draw(frame, detections)

    #         # Encode + stream
    #         chunk = self._encode(frame)
    #         if chunk:
    #             yield chunk

    # # -----------------------------
    # # STREAM 3 — TRACKING ONLY
    # # -----------------------------
    # def _gen_track(self):
        
    #     # Get queues
    #     q_rgb = self.device.q_rgb
    #     q_nn  = self.device.q_nn

    #     # Get frame + detections
    #     while True:
    #         frame = q_rgb.get().getCvFrame()
    #         in_nn = q_nn.tryGet()
    #         detections = in_nn.detections if in_nn is not None else []

    #         # Run tracking
    #         tracks = self.tracker.update(detections, frame.shape)

    #         # Update ROS2 publisher
    #         node = self.ros_holder.get("node")
    #         if node is not None:
    #             node.latest_tracks = tracks

    #         # Draw tracking boxes + IDs
    #         frame = self.tracker.draw_tracks(frame, tracks)

    #         # Encode + stream
    #         chunk = self._encode(frame)
    #         if chunk:
    #             yield chunk

    # # -----------------------------
    # # STREAM 4 — YOLO + TRACKING COMBINED
    # # -----------------------------
    # def _gen_unified(self):
        
    #     # Get queues
    #     q_rgb = self.device.q_rgb
    #     q_nn  = self.device.q_nn

    #     # Get RGB frame
    #     while True:
    #         frame = q_rgb.get().getCvFrame()

    #         # Get YOLO detections
    #         in_nn = q_nn.tryGet()
    #         detections = in_nn.detections if in_nn is not None else []

    #         # Run tracking
    #         tracks = self.tracker.update(detections, frame.shape)

    #         # Update ROS2
    #         node = self.ros_holder.get("node")
    #         if node is not None:
    #             node.latest_tracks = tracks

    #         # Draw YOLO boxes
    #         frame = self.yolo_drawer.draw(frame, detections)

    #         # Draw tracking IDs
    #         frame = self.tracker.draw_tracks(frame, tracks)

    #         # Encode + stream
    #         chunk = self._encode(frame)
    #         if chunk:
    #             yield chunk

    # -----------------------------
    # START Flask SERVER
    # -----------------------------
    def start(self):
        app.run(host="0.0.0.0", port=5000, threaded=True)
