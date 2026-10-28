# Standard library
import json                         # Convert tracking dictionaries to JSON for ROS2 publishing.
import threading                    # Run ROS2 in a background thread while the Flask server runs in the main thread.
import time

# Third-party libaries
import cv2                          # JPEG encoding and box drawing.
import numpy as np
from flask import Flask, Response, render_template_string   # HTTP endpoints, stream MJPEG frames.

# ROS 2
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node         # ROS2 node for publishing tracking data.
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String     # Tracking data published as JSON string messages.
from sensor_msgs.msg import LaserScan   # LiDAR scan measurements from /scan

# Project modules
from processors import DepthProcessor

# Create Flask app.
app = Flask(__name__)

# -----------------------------
# ROS2 Publisher
# -----------------------------
class RosTrackPublisher(Node):
    
    # Initialize ROS2 node.
    def __init__(self, context = None):
        
        # Node name is 'oakd_tracks', which will be used in ROS2 to identify this node.
        super().__init__('oakd_tracks', context = context)
        
        # Create ROS2 publisher. 
        # - Topic: /tracked_objects
        # - Message type: String
        # - Queue size: 10.
        self.pub = self.create_publisher(String, 'tracked_objects', 10)
        
        # Store the latest tracking data to be published.
        self.latest_tracks = [] 
        
        # Store the most recent LiDAR scan for the Flask dashboard
        self.latest_scan = None
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )
        
        # Create ROS2 timer to call timer_callback every 0.05 seconds (20 Hz).
        self.timer = self.create_timer(0.05, self.timer_callback)   

    # Keep the latest LiDAR scan available to the dashboard
    def scan_callback(self, msg):
        self.latest_scan = {
            "ranges": list(msg.ranges),
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_min),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
        }
        
    # Publish tracking data at 20Hz on the /tracked_objects topic. 
    # The data is converted to JSON string format before publishing.
    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.latest_tracks)
        self.pub.publish(msg)

# Run ROS2 publisher in a background thread so that it doesn't block the Flask server.
def start_ros_publisher(publisher_holder):

    # Share a shutdown signal with FlaskServer so the ROS thread can stop cleanly.
    stop_event = publisher_holder.setdefault("stop_event", threading.Event())
    context = Context()
    node = None
    executor = None

    try:
        # Give this publisher its own ROS 2 context.
        rclpy.init(
            context = context,
            signal_handler_options = SignalHandlerOptions.NO
        )
        
        node = RosTrackPublisher(context = context)      # Create ROS2 node
        publisher_holder["node"] = node # Store node in a shared dictionary to update `latest_tracks` from the Flask server.
    
        executor = SingleThreadedExecutor(context = context)
        executor.add_node(node)
    
        # Check the shutdown signal regularly.
        while context.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec = 0.1)

    finally:
        # Stop exposing the node to the processing loop.
        publisher_holder.pop("node", None)
    
        if executor is not None:
            
            if node is not None:
                executor.remove_node(node)
                
            executor.shutdown()
    
        # Release the node's resources.
        if node is not None:
            executor.destroy_node()
    
        # Shut down only this publisher's context.
        if context.ok():
            context.shutdown()

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
        self.depth_processor = DepthProcessor()

        # Shared processed frames and JPEGs. JPEGs are encoded once and reused
        # by every browser client rather than encoded separately per client.
        self.latest_frames = {
            "rgb": None,
            "unified": None,
            "depth": None,
            "lidar": None,
        }
        self.latest_jpegs = {
            "rgb": None,
            "unified": None,
            "depth": None,
            "lidar": None,
        }

        # Limit dashboard image refresh to reduce CPU/network overhead.
        # Perception/tracking and ROS 2 publishing still run at camera rate.
        self.display_interval = 1.0 / 15
        
        # Tell Flask clients when a new display frame is available
        self.frame_number = 0
        self.frame_condition = threading.Condition()

        # Track how many clients are requesting each stream
        self.active_streams = {
            "rgb": 0,
            "unified": 0,
            "depth": 0,
            "lidar": 0,
        }
        self._stop_event = threading.Event()
        
        # Register the dashboard and stream endpoints.
        self._register_routes()

        self.processing_thread = threading.Thread(
            target = self._processing_loop,
            daemon = True,
        )
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
                        function createCameraPanel(viewer, index, view)
                {
                    const panel = document.createElement("div");

                    panel.className = "panel";

                    const controls = document.createElement("div");

                    controls.className = "panel-controls";

                    const select = document.createElement("select");

                    select.className = "view-select";

                    Object.keys(viewNames).forEach(function(viewType)
                    {
                        const option = document.createElement("option");

                        option.value = viewType;

                        option.textContent = viewNames[viewType];

                        if (viewType === view)
                        {
                            option.selected = true;
                        }

                        select.appendChild(option);
                    });

                    select.onchange = function()
                    {
                        selectedViews[index] = select.value;

                        updateViewer();
                    };

                    controls.appendChild(select);

                    const image =
                        document.createElement("img");

                    image.src = "/" + view;

                    image.alt =
                        viewNames[view] + " Camera Stream";

                    panel.appendChild(controls);

                    panel.appendChild(image);

                    viewer.appendChild(panel);
                }

                updateViewer()

            </script>


        </body>

        </html>
            """)
        
        
        # Raw RGB frames
        @app.route("/rgb")
        def rgb_stream():
            return Response(
                self._gen_shared("rgb"),
                content_type="multipart/x-mixed-replace; boundary=frame",
            )

        # Unified YOLO + tracker view (one box per tracked object).
        @app.route("/unified")
        def unified_stream():
            return Response(
                self._gen_shared("unified"),
                content_type="multipart/x-mixed-replace; boundary=frame",
            )

        # Colorized stereo depth view.
        @app.route("/depth")
        def depth_stream():
            return Response(
                self._gen_shared("depth"),
                content_type="multipart/x-mixed-replace; boundary=frame",
            )

        # Live LiDAR scan stream from ROS 2 topic /scan.
        @app.route("/lidar")
        def lidar_stream():
            return Response(
                self._gen_shared("lidar"),
                content_type = "multipart/x-mixed-replace; boundary=frame",
                )

    def _make_lidar_frame(self, scan):
        """Render a LaserScan message as a top-down 2D point visualization."""
        width = 512
        height = 512
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:] = (12, 12, 12)
        center = (width // 2, height // 2)
        radius_px = 205

        if scan is None:
            cv2.putText(
                image, "Waiting for /scan data", (95, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA,
            )
            return image

        ranges = np.asarray(scan["ranges"], dtype=np.float32)
        range_min = max(0.0, float(scan["range_min"]))
        sensor_range_max = float(scan["range_max"])
        if not np.isfinite(sensor_range_max) or sensor_range_max <= 0:
            sensor_range_max = 8.0
        display_range = min(sensor_range_max, 8.0)
        display_range = max(display_range, 0.1)
        scale = radius_px / display_range

        # Draw distance rings and crosshairs.
        ring_count = 4
        for ring_index in range(1, ring_count + 1):
            distance = display_range * ring_index / ring_count
            ring_radius = int(distance * scale)
            cv2.circle(image, center, ring_radius, (55, 70, 70), 1)
            cv2.putText(
                image, f"{distance:.1f}m", (center[0] + 5, center[1] - ring_radius + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (130, 145, 145), 1, cv2.LINE_AA,
            )

        cv2.line(image, (center[0] - radius_px, center[1]),
                 (center[0] + radius_px, center[1]), (45, 55, 55), 1)
        cv2.line(image, (center[0], center[1] - radius_px),
                 (center[0], center[1] + radius_px), (45, 55, 55), 1)

        # ROS LaserScan angle 0 points along +X; display it toward the top.
        angle_min = float(scan["angle_min"])
        angle_increment = float(scan["angle_increment"])
        angles = angle_min + np.arange(ranges.size, dtype=np.float32) * angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= range_min)
            & (ranges <= display_range)
            & (ranges > 0.0)
        )
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        if valid_ranges.size:
            px = np.rint(center[0] - valid_ranges * np.sin(valid_angles) * scale).astype(int)
            py = np.rint(center[1] - valid_ranges * np.cos(valid_angles) * scale).astype(int)
            inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
            for point_x, point_y in zip(px[inside], py[inside]):
                cv2.circle(image, (int(point_x), int(point_y)), 2, (0, 220, 100), -1)

        cv2.circle(image, center, 6, (0, 190, 255), -1)
        cv2.putText(
            image, f"LiDAR /scan | displayed range: {display_range:.1f} m",
            (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA,
        )
        cv2.putText(
            image, f"Valid returns: {int(valid_ranges.size)} / {ranges.size}",
            (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA,
        )
        return image

    # -----------------------------
    # JPEG ENCODER
    # -----------------------------
    def _encode_jpeg(self, frame):
        ret, jpeg = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 80]
        )

        if not ret:
            return None

        return jpeg.tobytes()

    # -----------------------------
    # SHARED PROCESSING LOOP
    # -----------------------------
    def _processing_loop(self):
        
        detections = []
        
        last_display_time = 0.0
        last_processing_debug_time = 0.0
        last_display_debug_time = 0.0
        
        latest_raw_depth = None

        while not self._stop_event.is_set():

            # Measure acquisition and perception processing time.
            loop_start = time.monotonic()
            
            # Keep retrieving raw RGB and depth for tracking.
            frame = self.device.get_rgb_frame()
            raw_depth = self.device.get_depth_frame()
            
            if raw_depth is not None:
                latest_raw_depth = raw_depth

            if frame is None:
                time.sleep(0.001)
            else:
                # Use the newest available detection packet.
                in_nn = self.device.q_nn.tryGet()
                
                if in_nn is not None:
                    detections = in_nn.detections

                # Keep tracking and distance updates running.
                tracks = self.tracker.update(detections, frame.shape)

                for t in tracks:
                    t["distance_m"] = self.depth_processor.distance_m(
                        latest_raw_depth, t, frame.shape
                    )
                    
                node = self.ros_holder.get("node")
                
                if node is not None:
                    node.latest_tracks = tracks

                # Diagnose acquisition, tracking, and ROS processing.
                processing_elapsed = time.monotonic() - loop_start
                debug_now = time.monotonic()

                # Report slow iterations at most once per second.
                if (
                    processing_elapsed > self.display_interval
                    and debug_now - last_processing_debug_time >= 1.0
                ):
                    print(
                        f"Acquisition/tracking/ROS took "
                        f"{processing_elapsed * 1000:.0f} ms)"
                        f"(target: {self.display_interval * 1000:.0f} ms)"
                    )
                    
                    last_processing_debug_time = debug_now
                    
                # Limit dashboard frame generation to the configured rate.
                now = time.monotonic()
                
                if now - last_display_time >= self.display_interval:

                    # Start measuring dashboard rendering and encoding time
                    display_start = time.monotonic()
                    
                    # Read the ative viewer counts under the shared lock.
                    with self.frame_condition:
                        active_rgb = self.active_streams["rgb"] > 0
                        active_unified = self.active_streams["unified"] > 0
                        active_depth = self.active_streams["depth"] > 0
                        active_lidar = self.active_streams["lidar"] > 0

                    # Render only the views currently being requested.   
                    rgb_frame = frame.copy() if active_rgb else None

                    unified_frame = None

                    if active_unified:
                        
                        unified_frame = self.tracker._draw_unified(
                            frame.copy(),
                            tracks
                        )

                    depth_frame = None

                    if active_depth:
                        
                        depth_frame = self.depth_processor.process(
                            latest_raw_depth
                        )
                    
                    lidar_frame = None
                    if active_lidar:
                        current_node = self.ros_holder.get("node")
                        latest_scan = current_node.latest_scan if current_node is not None else None
                        lidar_frame = self._make_lidar_frame(latest_scan)
                        
                    # Encode each requested view once for all its clients.
                    rgb_jpeg = None

                    if rgb_frame is not None:
                        rgb_jpeg = self._encode_jpeg(rgb_frame)

                    unified_jpeg = None
                    
                    if unified_frame is not None:
                        unified_jpeg = self._encode_jpeg(unified_frame)

                    
                    depth_jpeg = None
                    
                    if depth_frame is not None:
                        depth_jpeg = self._encode_jpeg(depth_frame)
                        
                    lidar_jpeg = None
                    
                    if lidar_frame is not None:
                        lidar_jpeg = self._encode_jpeg(lidar_frame)

                    # Diagnose slow rendering and JPEG encoding.
                    display_elapsed = time.monotonic() - display_start
                    debug_now = time.monotonic()
                    
                    # Report unusually slow dashpboard updates.
                    if display_elapsed > self.display_interval:
                        print(
                            f"Dashboard rendering/encoding took "
                            f" {display_elapsed * 1000:.1f} ms"
                            f"(target: {self.display_interval * 1000:.0f} ms)"
                        )

                    last_display_debug_time = debug_now
                        
                    # Publish the latest frames and wake waiting stream clients.
                    with self.frame_condition:
                        self.latest_frames = {
                            "rgb": rgb_frame,
                            "unified": unified_frame,
                            "depth": depth_frame,
                            "lidar": lidar_frame,
                        }
                        
                        self.latest_jpegs = {
                            "rgb": rgb_jpeg,
                            "unified": unified_jpeg,
                            "depth": depth_jpeg,
                            "lidar": lidar_jpeg,
                        }
                        
                        self.frame_number += 1
                        self.frame_condition.notify_all()

                    last_display_time = now

    # SHARED MJPEG GENERATOR
    def _gen_shared(self, stream_type):
        
        # Keep track of which frame the browser has already recieved
        last_frame_number = -1

        # Register this client as an active viewer.
        with self.frame_condition:
            
            self.active_streams[stream_type] += 1

        try:
            while not self._stop_event.is_set():

            
                with self.frame_condition:
                    
                    # Wait until a new frame is available or shutdown begins.
                    self.frame_condition.wait_for(
                        lambda: (
                            self.frame_number != last_frame_number
                            or self._stop_event.is_set()
                        )
                    )

                    if self._stop_event.is_set():
                        return
                    
                    jpeg_bytes = self.latest_jpegs.get(stream_type)

                    current_frame_number = self.frame_number

                last_frame_number = current_frame_number

                if jpeg_bytes is not None:

                    chunk = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " 
                        + str(len(jpeg_bytes)).encode() 
                        + b"\r\n\r\n"
                        + jpeg_bytes
                        + b"\r\n"
                    )
        
                    yield chunk

        finally:
            
            # Unregister the client when its stream ends.
            with self.frame_condition:
                self.active_streams[stream_type] = max(
                    0, self.active_streams[stream_type] -1
                )

                self.frame_condition.notify_all()

    def stop(self):
        self._stop_event.set()

        ros_stop_event = self.ros_holder.get("stop_event")
        
        if ros_stop_event is not None:
            ros_stop_event.set()
    
    # -----------------------------
    # START Flask SERVER
    # -----------------------------
    def start(self):
        app.run(host = "0.0.0.0", port = 5000, threaded = True)
