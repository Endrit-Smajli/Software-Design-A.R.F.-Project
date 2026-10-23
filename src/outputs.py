from flask import Flask, Response, render_template_string   # HTTP endpoints, stream MJPEG frames.
import cv2                          # JPEG encoding and box drawing.
import time
import json                         # Convert tracking dictionaries to JSON for ROS2 publishing.
import threading                    # Run ROS2 in a background thread while the Flask server runs in the main thread.
import numpy as np
from processors import DepthProcessor

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
        self.depth_processor = DepthProcessor()

        # Shared processed frames and JPEGs. JPEGs are encoded once and reused
        # by every browser client rather than encoded separately per client.
        self.latest_frames = {
            "rgb": None,
            "unified": None,
            "depth": None,
        }
        self.latest_jpegs = {
            "rgb": None,
            "unified": None,
            "depth": None,
        }

        # LiDAR is a static placeholder until the actual sensor/topic is connected.
        self.lidar_placeholder = self._make_lidar_placeholder()
        self.lidar_placeholder_jpeg = self._encode_jpeg(self.lidar_placeholder)

        # Limit dashboard image refresh to reduce CPU/network overhead.
        # Perception/tracking and ROS 2 publishing still run at camera rate.

        self.display_interval = 1.0 / 15.0


        # Tell Flask clients when a new display frame is available.
        self.frame_number = 0
        self.frame_condition = threading.Condition()

        # Track how many clients are requesting each stream
        self.active_streams = {
            "rgb": 0,
            "unified": 0,
            "depth": 0,
        }
        self._stop_event = threading.Event()
        
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

                A.R.F Camera Dashboard (Front, OAK-D Lite)

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

        # Static placeholder until the LiDAR ROS 2 topic is connected.
        # Returning one JPEG avoids repeatedly transmitting an unchanged image.
        @app.route("/lidar")
        def lidar_stream():
            return Response(self.lidar_placeholder_jpeg, mimetype="image/jpeg")

    def _make_lidar_placeholder(self):
        """Create a clear placeholder while LiDAR input is not connected."""
        image = np.zeros((288, 512, 3), dtype=np.uint8)
        image[:] = (12, 12, 12)

        center = (256, 144)
        for radius in (35, 70, 105):
            cv2.circle(image, center, radius, (55, 70, 70), 1)

        cv2.line(image, (151, 144), (361, 144), (55, 70, 70), 1)
        cv2.line(image, (256, 39), (256, 249), (55, 70, 70), 1)
        cv2.circle(image, center, 5, (0, 190, 255), -1)
        cv2.putText(
            image,
            "LiDAR",
            (18, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (230, 230, 230),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "Integration pending",
            (145, 275),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (170, 170, 170),
            1,
            cv2.LINE_AA,
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
                        }
                        
                        self.latest_jpegs = {
                            "rgb": rgb_jpeg,
                            "unified": unified_jpeg,
                            "depth": depth_jpeg,
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

        # Wake stream generators that are waiting for another frame.
        with self.frame_condition:
            self.frame_condition.notify_all()
    
        self.processing_thread.join(timeout = 3)
    # -----------------------------
    # START Flask SERVER
    # -----------------------------
    def start(self):
        app.run(host = "0.0.0.0", port = 5000, threaded = True)
