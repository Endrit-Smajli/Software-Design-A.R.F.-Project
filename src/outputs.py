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
    STREAM_NAMES = ("rgb", "unified", "depth", "lidar")
    STREAM_FPS = {"rgb": 15, "unified": 15, "depth": 15, "lidar": 10}
    STREAM_QUALITY = {"rgb": 70, "unified": 70, "depth": 50, "lidar": 70}

    def __init__(self, device, yolo_drawer, tracker, ros_holder):

        self.device = device
        self.yolo_drawer = yolo_drawer
        self.tracker = tracker
        self.ros_holder = ros_holder
        self.depth_processor = DepthProcessor()

        # Cached static LiDAR grid, rebuilt only if the display range changes.
        self._lidar_bg = None
        self._lidar_bg_range = None

        # Newest acquisition result: (frame, tracks, raw_depth, scan).
        # Written by the processing loop, read by the per-view workers.
        self.latest_source = None

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
        
        # Register the dashboard and stream endpoints.
        self._register_routes()

        self.processing_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True,
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
        <!DOCTYPE html>

        <html>

        <head>

            <meta name = "viewport"
                content = "width=device-width, initial-scale = 1.0">
            <title>A.R.F Camera Dashboard</title>

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

                /* --------------------------------
                   Header
                -------------------------------- */

                .header {
                    height: 60px;

                    display: flex;
                    align-items: center;

                    padding: 0 16px;

                    background: #1b1b1b;

                    font-size: 20px;
                    font-weight: bold;
                }


                /* --------------------------------
                   Control buttons
                -------------------------------- */

                .controls {
                    height: 55px;

                    display: flex;
                    align-items: center;

                    gap: 10px;

                    padding: 8px 16px;

                    background: #222;
                }

                .controls button {

                    padding: 8px 16px;

                    border: none;
                    border-radius: 4px;

                    background: #444;
                    color: white;

                    cursor: pointer;

                    font-size: 14px;
                }

                .controls button:hover {
                    background: #555;
                }

                .controls button.active {
                    background: #666;
                }


                /* --------------------------------
                   Display area
                -------------------------------- */

                .viewer {

                    width: 100%;
                    height: calc(100vh - 115px);

                    display: grid;
                    gap: 5px;

                    background: black;
                    padding: 6px;

                    overflow: hidden;
                    box-sizing: border-box;
                }


                /* --------------------------------
                   Individual display panels
                -------------------------------- */

                .panel {

                    position: relative;

                    min-width: 0;
                    min-height: 0;

                    background: black;

                    display: flex;
                    align-items: center;
                    justify-content: center;

                    overflow: hidden;
                }

                .panel img {
                    width: 100%;
                    height: 100%;

                    object-fit: contain;

                    display: block;
                }
                
                /* --------------------------------
                   Individual control panels
                -------------------------------- */
                .panel-controls {
                    position: absolute;

                    top: 8px;

                    right: 8px;

                    z-index: 10;
                }

                .view-select {

                    background-color: #34495e;
                    color: white;

                    border: 1px solid #607d8b;
                    border-radius: 6px;

                    padding: 7px 12px;

                    font-size: 14px;
                    font-weight: 500;

                    cursor: pointer;
                }

                .view-select option {
                    background-color: #34495e;
                    color: white;
                }
                
                /* --------------------------------
                   Camera image
                -------------------------------- */

                .panel img {

                    width: 100%;
                    height: 100%;

                    object-fit: contain;

                    display: block;
                }

                @media (max-width: 768px) {

                    .viewer {
                        height: calc(100vh - 130px);

                        display: grid;

                        grid-template-columns: 1fr !important;
                        grid-template-rows: none !important;

                        grid-auto-rows: minmax(45vh, auto) !important;
                        
                        gap: 4px;
                        padding: 4px;

                        overflow-y: auto;
                    }

                    .panel {
                        width: 100%;
                        min-width: 0;
                        min-height: 45vh;
                    }

                    .panel-controls {
                        top: 6px;
                        left: 6px;
                    }

                    .view-select {
                        font-size: 14px;
                        padding: 6px 8px;
                    }
                }
                    
                /* --------------------------------
                   Display label
                -------------------------------- */

                .label {

                    position: absolute;

                    top: 10px;
                    left: 10px;

                    padding: 6px 10px;

                    background: rgba(0, 0, 0, 0.7);

                    border-radius: 4px;

                    font-size: 14px;
                    font-weight: bold;
                }


                /* --------------------------------
                   LiDAR placeholder
                -------------------------------- */

                .lidar-placeholder {

                    width: 100%;
                    height: 100%;

                    display: flex;

                    align-items: center;
                    justify-content: center;

                    background: #050505;

                    color: #888;

                    font-size: 24px;
                }

            </style>

            

        </head>


        <body>


            <!-- Header -->

            <div class="header">

                A.R.F Multi-View Dashboard 

            </div>


            <!-- Buttons -->

            <div class="controls">

                <span class = "control-title">Number of Views:

                </span>

                <button onclick = "setViewCount(1, this)" class = "active">
                    1
                </button>

                <button onclick = "setViewCount(2, this)">
                    2
                </button>
                
                <button onclick = "setViewCount(3, this)">
                    3
                </button>

                <button onclick = "setViewCount(4, this)">
                    4
                </button>
                
            </div>


            <!-- Display area -->

            <div
                id = "viewer"
                class = "viewer">

            </div>


            <script>

                const viewNames = {
                    rgb: "RGB",
                    unified: "Unified",
                    depth: "Depth",
                    lidar: "LiDAR"
                };

                let viewCount = 1;

                let selectedViews = [
                    "rgb",
                    "unified",
                    "depth",
                    "lidar"
                ];

                function setViewCount(count, button)
                {
                    viewCount = count;

                    const buttons = document.querySelectorAll(".controls button");

                    buttons.forEach(function(item)
                    {
                        item.classList.remove("active");
                    });

                    button.classList.add("active");

                    updateViewer();
                }

                function updateViewer()
                {
                    const viewer = document.getElementById("viewer");

                    // Close the old MJPEG connections before removing the images.
                    viewer.querySelectorAll("img").forEach(function(img)
                    {
                        img.src = "";
                    });

                    viewer.innerHTML = "";

                    if (viewCount === 1)
                    {
                        viewer.style.gridTemplateColumns = "1fr";
                        viewer.style.gridTemplateRows = "1fr";
                    }
                    else if (viewCount === 2)
                    {
                        viewer.style.gridTemplateColumns = "1fr 1fr";
                        viewer.style.gridTemplateRows = "1fr";
                    }
                    else
                    {
                        viewer.style.gridTemplateColumns = "1fr 1fr";
                        viewer.style.gridTemplateRows = "1fr 1fr";
                    }

                    for (let i = 0; i < viewCount; i++)
                    {
                        createCameraPanel(
                            viewer,
                            i,
                            selectedViews[i]
                        );
                    }
                }

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
        last_processing_debug_time = 0.0
        latest_raw_depth = None

        while not self._stop_event.is_set():

            loop_start = time.monotonic()

            frame = self.device.get_rgb_frame()
            raw_depth = self.device.get_depth_frame()

            if raw_depth is not None:
                latest_raw_depth = raw_depth

            if frame is None:
                time.sleep(0.001)
                continue

            # Use the newest available detection packet.
            in_nn = self.device.q_nn.tryGet()

            if in_nn is not None:
                detections = in_nn.detections

            tracks = self.tracker.update(detections, frame.shape)
            tracks = [dict(t) for t in tracks]    # private copy for the workers

            for t in tracks:
                t["distance_m"] = self.depth_processor.distance_m(
                    latest_raw_depth, t, frame.shape
                )

            node = self.ros_holder.get("node")
            scan = None

            if node is not None:
                node.latest_tracks = tracks
                scan = node.latest_scan

            # Hand the newest data to the per-view workers.
            self.latest_source = (frame, tracks, latest_raw_depth, scan)

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
                        
    def _render_stream(self, name, frame, tracks, raw_depth, scan):
        """Build the BGR image for one view from the newest source data."""
        if name == "rgb":
            return frame

        if name == "unified":
            return self.tracker._draw_unified(frame.copy(), tracks)

        if name == "depth":
            return self.depth_processor.process(raw_depth)

        if name == "lidar":
            return self._make_lidar_frame(scan)

        return None

    def _stream_worker(self, name):
        """Render and encode one view, but only while someone is watching it."""
        min_interval = 1.0 / self.STREAM_FPS[name]
        quality = self.STREAM_QUALITY[name]
        condition = self.stream_conditions[name]

        no_key = object()
        last_key = no_key
        last_run = 0.0
        last_warn = 0.0

        while not self._stop_event.is_set():

            source = self.latest_source

            # Idle while no browser is viewing this stream.
            if self.active_streams[name] == 0 or source is None:
                time.sleep(0.05)
                continue

            frame, tracks, raw_depth, scan = source

            # The input object that tells us this view has new data.
            key = {
                "rgb": frame,
                "unified": frame,
                "depth": raw_depth,
                "lidar": scan,
            }[name]

            now = time.monotonic()

            if key is last_key or now - last_run < min_interval:
                time.sleep(0.003)
                continue

            last_key = key
            last_run = now

            start = time.monotonic()
            image = self._render_stream(name, frame, tracks, raw_depth, scan)
            jpeg = self._encode_jpeg(image, quality) if image is not None else None
            elapsed = time.monotonic() - start

            if elapsed > min_interval and start - last_warn >= 1.0:
                print(
                    f"{name} view took {elapsed * 1000:.0f} ms "
                    f"(target: {min_interval * 1000:.0f} ms)"
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

                with condition:
                    condition.wait_for(
                        lambda: (
                            self.stream_numbers[stream_type] != last_number
                            or self._stop_event.is_set()
                        ),
                        timeout = 1.0,
                    )

                    if self._stop_event.is_set():
                        return

                    if self.stream_numbers[stream_type] == last_number:
                        continue    # timed out with nothing new

                    jpeg_bytes = self.latest_jpegs[stream_type]
                    last_number = self.stream_numbers[stream_type]

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
