# Standard library
import json                         # Convert tracking dictionaries to JSON for ROS2 publishing.
import threading                    # Run ROS2 in a background thread while the Flask server runs in the main thread.
import time

# Third-party libaries
import cv2                          # JPEG encoding and box drawing.
import numpy as np
from flask import Flask, Response, render_template   # HTTP endpoints, stream MJPEG frames.

# ROS 2
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node         # ROS2 node for publishing tracking data.
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String     # Tracking data published as JSON string messages.
from sensor_msgs.msg import LaserScan  # LiDAR scan measurements from /scan.

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

        # Store the most recent LiDAR scan for the Flask dashboard.
        self.latest_scan = None
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )
        
        # Create ROS2 timer to call timer_callback every 0.05 seconds (20 Hz).
        self.timer = self.create_timer(0.05, self.timer_callback)   

    # Keep the latest LiDAR scan available to the dashboard.
    def scan_callback(self, msg):
        self.latest_scan = {
            "ranges": np.array(msg.ranges, dtype = np.float32),
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment),
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
def start_ros_publisher(publisher_holder, stop_event=None):

    # Share a shutdown signal with main.py and FlaskServer so the ROS thread
    # can stop cleanly. Accept an optional event for compatibility with callers
    # that still pass (ros_holder, ros_stop_event).
    if stop_event is None:
        stop_event = publisher_holder.setdefault("stop_event", threading.Event())
    else:
        publisher_holder["stop_event"] = stop_event
    context = Context()
    node = None
    executor = None

    try:
        # Give this publisher its own ROS 2 context.
        rclpy.init(
            context = context,
            signal_handler_options = SignalHandlerOptions.NO
        )
        
        #rclpy.init()                    # Initialize ROS2 system
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
            node.destroy_node()
    
        # Shut down only this publisher's context.
        if context.ok():
            context.shutdown()

# -----------------------------
# Flask Server
# -----------------------------
class FlaskServer:
    
    # Constructor recieves and stores the OAK-D device, YOLO drawer, 
    # tracker, and ROS2 publisher holder for later use.
    STREAM_NAMES = ("rgb", "unified", "depth", "lidar")
    STREAM_FPS = {"rgb": 15, "unified": 15, "depth": 15, "lidar": 10}
    STREAM_QUALITY = {"rgb": 70, "unified": 70, "depth": 50, "lidar": 70}

    def __init__(self, device, tracker, ros_holder):

        self.device = device
        self.tracker = tracker
        self.ros_holder = ros_holder
        self.depth_processor = DepthProcessor()

        # Cached static LiDAR grid, rebuilt only if the display range changes.
        self._lidar_bg = None
        self._lidar_bg_range = None

        # Newest acquisition result: (frame, tracks, raw_depth, scan).
        # Written by the processing loop, read by the per-view workers.
        self.latest_source = None
        
        # Wakes the per-view workers when the processing loop publishes new data.
        self.source_condition = threading.Condition()
        self.source_version = 0

        # Each view has its own JPEG slot, frame counter and condition,
        # so a slow view can never hold up the others.
        self.latest_jpegs = {name: None for name in self.STREAM_NAMES}
        self.stream_numbers = {name: 0 for name in self.STREAM_NAMES}
        self.stream_conditions = {
            name: threading.Condition() for name in self.STREAM_NAMES
        }

        # Number of browser clients watching each view.
        self.active_streams = {name: 0 for name in self.STREAM_NAMES}
        self.active_lock = threading.Lock()

        self._stop_event = threading.Event()

        # Register the dashboard and stream endpoints.
        self._register_routes()

        self.processing_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True,
        )
        self.processing_thread.start()

        # One render/encode worker per view.
        self.stream_threads = [
            threading.Thread(target=self._stream_worker, args=(name,), daemon=True)
            for name in self.STREAM_NAMES
        ]
        for thread in self.stream_threads:
            thread.start()
    
    # --------------------------------------------------------
    # ROUTES - Define all HTTP endpoints for the Flask server.
    # --------------------------------------------------------
    def _register_routes(self):
        
        # Main page with links to all streams
        @app.route("/")
        @app.route("/dashboard")
        
        def dashboard():
            return render_template("dashboard.html")
        
        
        # Raw RGB frames
        @app.route("/rgb")
        def rgb_stream():
            return self._mjpeg_response("rgb")

        # Unified YOLO + tracker view (one box per tracked object).
        @app.route("/unified")
        def unified_stream():
            return self._mjpeg_response("unified")

        # Colorized stereo depth view.
        @app.route("/depth")
        def depth_stream():
            return self._mjpeg_response("depth")

        # Live LiDAR scan stream from ROS 2 topic /scan.
        @app.route("/lidar")
        def lidar_stream():
            return self._mjpeg_response("lidar")

    def _mjpeg_response(self, name):
        resp = Response(
            self._gen_shared(name),
            content_type="multipart/x-mixed-replace; boundary=frame",
        )
        resp.headers["Cache-Control"] = "no-store"
        resp.headers["X-Accel-Buffering"] = "no"
        return resp
    
    def _draw_lidar_grid(self, display_range):
        """Draw the static LiDAR background (grid, axes, labels) once."""
        width = 512
        height = 512
        image = np.full((height, width, 3), (18, 18, 18), dtype=np.uint8)
        center_x = width // 2
        center_y = height // 2
        grid_radius_px = 210
        pixels_per_metre = grid_radius_px / display_range

        def world_to_pixel(x_metres, y_metres):
            px = int(round(center_x + x_metres * pixels_per_metre))
            py = int(round(center_y - y_metres * pixels_per_metre))
            return px, py

        # Draw a half-metre minor grid and a one-metre major grid.
        step_m = 0.5
        grid_steps = int(np.floor(display_range / step_m))
        for step_index in range(-grid_steps, grid_steps + 1):
            coordinate = step_index * step_m
            is_major = step_index % 2 == 0
            grid_colour = (52, 52, 52) if is_major else (34, 34, 34)
            thickness = 1

            vertical_x, _ = world_to_pixel(coordinate, 0.0)
            horizontal_y = world_to_pixel(0.0, coordinate)[1]
            cv2.line(
                image,
                (vertical_x, center_y - grid_radius_px),
                (vertical_x, center_y + grid_radius_px),
                grid_colour,
                thickness,
                cv2.LINE_AA,
            )
            cv2.line(
                image,
                (center_x - grid_radius_px, horizontal_y),
                (center_x + grid_radius_px, horizontal_y),
                grid_colour,
                thickness,
                cv2.LINE_AA,
            )

        # Highlight the ROS X/Y axes (X red, Y green) and border the grid.
        grid_left = center_x - grid_radius_px
        grid_right = center_x + grid_radius_px
        grid_top = center_y - grid_radius_px
        grid_bottom = center_y + grid_radius_px
        cv2.line(image, (grid_left, center_y), (grid_right, center_y), (0, 0, 150), 1, cv2.LINE_AA)
        cv2.line(image, (center_x, grid_top), (center_x, grid_bottom), (0, 125, 0), 1, cv2.LINE_AA)
        cv2.rectangle(image, (grid_left, grid_top), (grid_right, grid_bottom), (105, 105, 105), 1)

        # Label the grid border every two metres to avoid clutter.
        label_step = 2.0
        label_steps = int(np.floor(display_range / label_step))
        for step_index in range(-label_steps, label_steps + 1):
            coordinate = step_index * label_step
            px, _ = world_to_pixel(coordinate, 0.0)
            _, py = world_to_pixel(0.0, coordinate)
            label = f"{coordinate:g}"
            cv2.putText(
                image, label, (px - 10, grid_bottom + 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (160, 160, 160), 1, cv2.LINE_AA,
            )
            if step_index != 0:
                cv2.putText(
                    image, label, (grid_left - 25, py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (160, 160, 160), 1, cv2.LINE_AA,
                )

        cv2.putText(
            image, "X (m)", (grid_right - 38, grid_bottom + 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 220), 1, cv2.LINE_AA,
        )
        cv2.putText(
            image, "+Y", (center_x + 7, grid_top + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 220, 120), 1, cv2.LINE_AA,
        )
        cv2.putText(
            image, "+X", (grid_right - 27, center_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 220), 1, cv2.LINE_AA,
        )

        return image
    def _make_lidar_frame(self, scan):
        """Render /scan in an RViz-style, top-down Cartesian grid.

        The grid is centered on the robot and uses the ROS convention:
        +X points right, +Y points up, and angles are measured counterclockwise
        from +X. This is a live scan view, not a persistent occupancy map.
        """
        width = 512
        height = 512
        center_x = width // 2
        center_y = height // 2
        center = (center_x, center_y)
        grid_radius_px = 210

        if scan is None:
            image = np.full((height, width, 3), (18, 18, 18), dtype=np.uint8)
            cv2.putText(
                image,
                "Waiting for /scan data",
                (82, height // 2 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (190, 190, 190),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                "Check that the ROS 2 subscription is receiving LaserScan",
                (24, height // 2 + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.36,
                (130, 130, 130),
                1,
                cv2.LINE_AA,
            )
            return image

        ranges = np.asarray(scan.get("ranges", []), dtype=np.float32)
        range_min = float(scan.get("range_min", 0.0))
        sensor_range_max = float(scan.get("range_max", 8.0))
        if not np.isfinite(range_min):
            range_min = 0.0
        if not np.isfinite(sensor_range_max) or sensor_range_max <= 0.0:
            sensor_range_max = 8.0

        # The plotted square spans +/- display_range metres on each axis.
        # Range returns longer than this are omitted to keep the view readable.
        display_range = max(0.5, min(sensor_range_max, 8.0))
        pixels_per_metre = grid_radius_px / display_range

        # Bounds of the plotted grid, used to discard points outside it.
        grid_left = center_x - grid_radius_px
        grid_right = center_x + grid_radius_px
        grid_top = center_y - grid_radius_px
        grid_bottom = center_y + grid_radius_px

        # Reuse the pre-drawn grid instead of redrawing it every frame.
        if self._lidar_bg is None or self._lidar_bg_range != display_range:
            self._lidar_bg = self._draw_lidar_grid(display_range)
            self._lidar_bg_range = display_range
        image = self._lidar_bg.copy()

        # Convert each LaserScan polar measurement into ROS X/Y coordinates.
        angle_min = float(scan.get("angle_min", 0.0))
        angle_increment = float(scan.get("angle_increment", 0.0))
        if not np.isfinite(angle_min):
            angle_min = 0.0
        if not np.isfinite(angle_increment):
            angle_increment = 0.0

        angles = angle_min + np.arange(ranges.size, dtype=np.float32) * angle_increment
        valid = (
            np.isfinite(ranges)
            & np.isfinite(angles)
            & (ranges >= max(0.0, range_min))
            & (ranges > 0.0)
            & (ranges <= display_range)
        )
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        if valid_ranges.size:
            # ROS coordinates: X = r*cos(angle), Y = r*sin(angle).
            point_x = center_x - valid_ranges * np.cos(valid_angles) * pixels_per_metre
            point_y = center_y + valid_ranges * np.sin(valid_angles) * pixels_per_metre
            point_x = np.rint(point_x).astype(np.int32)
            point_y = np.rint(point_y).astype(np.int32)
            inside = (
                (point_x >= grid_left)
                & (point_x <= grid_right)
                & (point_y >= grid_top)
                & (point_y <= grid_bottom)
            )

            # Draw all points at once with NumPy indexing (2x2 dots)
            # instead of one cv2.circle call per point.
            h, w = image.shape[:2]
            xs, ys = point_x[inside], point_y[inside]
            for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
                image[np.clip(ys + dy, 0, h - 1), np.clip(xs + dx, 0, w - 1)] = (255, 0, 255)

        # Draw a robot-origin marker with its forward (+X) direction.
        cv2.circle(image, center, 7, (20, 20, 20), -1, cv2.LINE_AA)
        cv2.circle(image, center, 5, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.arrowedLine(
            image, (center_x, center_y), (center_x + 17, center_y),
            (0, 220, 255), 2, cv2.LINE_AA, tipLength=0.35,
        )

        cv2.putText(
            image,
            f"LaserScan | /scan | grid: +/-{display_range:.1f} m",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"Valid returns: {int(valid_ranges.size)} / {ranges.size}",
            (10, height - 9),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (175, 175, 175),
            1,
            cv2.LINE_AA,
        )
        return image

    # -----------------------------
    # JPEG ENCODER
    # -----------------------------
    def _encode_jpeg(self, frame, quality = 70):
        ret, jpeg = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, quality]
        )

        if not ret:
            return None

        return jpeg.tobytes()
    # -----------------------------
    # SHARED PROCESSING LOOP
    # -----------------------------
    def _processing_loop(self):

        detections = []
        tracks = []
        latest_frame = None
        latest_raw_depth = None
        last_processing_debug_time = 0.0

        while not self._stop_event.is_set():

            loop_start = time.monotonic()

            frame = self.device.get_rgb_frame()
            raw_depth = self.device.get_depth_frame()

            if raw_depth is not None:
                latest_raw_depth = raw_depth

            if frame is not None:
                latest_frame = frame

                # Use the newest available detection packet.
                in_nn = self.device.get_detections()

                if in_nn is not None:
                    detections = in_nn.detections

                tracks = self.tracker.update(detections, frame.shape)

            has_new_data = frame is not None or raw_depth is not None

            if has_new_data and latest_frame is not None:

                published = [dict(t) for t in tracks]    # private copy for the workers

                for t in published:
                    t["distance_m"] = self.depth_processor.distance_m(
                        latest_raw_depth, t, latest_frame.shape
                    )

                node = self.ros_holder.get("node")
                scan = None

                if node is not None:
                    node.latest_tracks = published
                    scan = node.latest_scan

                # Hand the newest data to the per-view workers.
                with self.source_condition:
                    self.latest_source = (latest_frame, published, latest_raw_depth, scan)
                    self.source_version += 1
                    self.source_condition.notify_all()

                # Diagnose acquisition, tracking, and ROS processing.
                processing_elapsed = time.monotonic() - loop_start
                debug_now = time.monotonic()

                if (
                    processing_elapsed > 1.0 / 15.0
                    and debug_now - last_processing_debug_time >= 1.0
                ):
                    print(
                        f"Acquisition/tracking/ROS took "
                        f"{processing_elapsed * 1000:.0f} ms"
                    )

                    last_processing_debug_time = debug_now
            else:
                time.sleep(0.005)
                        
    def _render_stream(self, name, frame, tracks, raw_depth, scan):
        """Build the BGR image for one view from the newest source data."""
        if name == "rgb":
            return frame

        if name == "unified":
            return self.tracker.draw_unified(frame.copy(), tracks)

        if name == "depth":
            return self.depth_processor.process(raw_depth)

        if name == "lidar":
            return self._make_lidar_frame(scan)

        return None

    def _stream_worker(self, name):
        """Render and encode one view, but only while someone is watching it."""
        
        frame_budget = 1.0 / self.STREAM_FPS[name]
        quality = self.STREAM_QUALITY[name]
        condition = self.stream_conditions[name]

        last_version = -1
        last_key = object()
        
        last_warn = 0.0

        while not self._stop_event.is_set():

            # Sleep until the processing loop publishes something new.
            with self.source_condition:
                self.source_condition.wait_for(
                    lambda: self.source_version != last_version
                    or self._stop_event.is_set(),
                    timeout = 0.5,
                )
                
                version = self.source_version
                source = self.latest_source
            
            new_data = version != last_version
            last_version = version
            watched = self.active_streams[name] > 0
            
            if new_data and watched and source is not None:
                frame, tracks, raw_depth, scan = source
                
                # The input object that tells us this view has new data.
                key = {
                    "rgb": frame,
                    "unified": frame,
                    "depth": raw_depth,
                    "lidar": scan,
                }[name]
                
                if key is not last_key:
                    last_key = key
                    
                    start = time.monotonic()
                    image = self._render_stream(name, frame, tracks, raw_depth, scan)
                    jpeg = self._encode_jpeg(image, quality) if image is not None else None
                    elapsed = time.monotonic() - start
                    
                    if elapsed > frame_budget and start - last_warn >= 1.0:
                        print(
                            f"{name} view took {elapsed * 1000:.0f} ms "
                            f"(target: {frame_budget * 1000:.0f} ms)"
                        )
                        
                        last_warn = start
                    
                    with condition:
                        self.latest_jpegs[name] = jpeg
                        self.stream_numbers[name] += 1
                        condition.notify_all()
                        
    # SHARED MJPEG GENERATOR
    def _gen_shared(self, stream_type):

        condition = self.stream_conditions[stream_type]
        last_number = -1

        # Register this client as an active viewer.
        with self.active_lock:
            self.active_streams[stream_type] += 1

        try:
            while not self._stop_event.is_set():
                
                jpeg_bytes = None
                
                with condition:
                    condition.wait_for(
                        lambda: (
                            self.stream_numbers[stream_type] != last_number
                            or self._stop_event.is_set()
                        ),
                        timeout = 1.0,
                    )

                    if (
                        not self._stop_event.is_set()
                        and self.stream_numbers[stream_type] != last_number
                    ):
                        jpeg_bytes = self.latest_jpegs[stream_type]
                        last_number = self.stream_numbers[stream_type]
                        
                    if self._stop_event.is_set():
                        return

                if jpeg_bytes is not None:

                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: "
                        + str(len(jpeg_bytes)).encode()
                        + b"\r\n\r\n"
                        + jpeg_bytes
                        + b"\r\n"
                    )

        finally:
            # Unregister the client when its stream ends.
            with self.active_lock:
                self.active_streams[stream_type] = max(
                    0, self.active_streams[stream_type] - 1
                )

    def stop(self):
        self._stop_event.set()
        ros_stop_event = self.ros_holder.get("stop_event")
        if ros_stop_event is not None:
            ros_stop_event.set()

        # Wake stream generators that are waiting for another frame.
        with self.source_condition:
            self.source_condition.notify_all()
            
        for condition in self.stream_conditions.values():
            with condition:
                condition.notify_all()

        self.processing_thread.join(timeout = 3)

        for thread in self.stream_threads:
            thread.join(timeout = 1)
            
    # -----------------------------
    # START Flask SERVER
    # -----------------------------
    def start(self):
        app.run(host = "0.0.0.0", port = 5000, threaded = True)
