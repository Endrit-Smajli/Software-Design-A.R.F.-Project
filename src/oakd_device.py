import depthai as dai   # DetphAi SDK import

class OakDDevice:
    """
    OAK-D Lite camera and YOLOv6 Nano object detection.
    
    Uses the DepthAI v3 API and Luxonis Model Zoo instead of
    the old YOLOv5 OpenVINO .blob file.
    """
    
    # Establish DetphAI pipeline
    def __init__(self):
        self.pipeline = dai.Pipeline()

        self._build_color_camera()  # Build color camera node in the pipeline
        self._build_yolo()          # Build YOLO node in the pipeline
        self._build_stereo_depth()  # Build stereo depth node in the pipeline
        
        #self.pipeline.start()  # Start the pipeline. This initializes the nodes and prepares them for execution.

        # Create output queues
        self.q_rgb = self.nn.passthrough.createOutputQueue(maxSize = 1, blocking = False)
        self.q_nn = self.nn.out.createOutputQueue(maxSize = 1, blocking = False)
        self.q_depth = self.stereo.depth.createOutputQueue(maxSize = 1, blocking = False)

        self.pipeline.start()  # Start the pipeline. This initializes the nodes and prepares them for execution.

    # ColorCamera node to capture RGB frames from OAK-D Lite sesor.
    def _build_color_camera(self):
        """
        Build the OAK-D Lite RGB camera.
        
        YOLOv6 Nano uses a 512x288 input in the current Luxonis DetectionNetwork example.
        """
        
        self.cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        # Request BGR frames at the model input resolution.

        #Comented out next line (do not delete)
        #self.cam_output = self.cam.requestOutput((512, 288), type = dai.ImgFrame.Type.BGR888p, fps = 30)

    # YOLO detection
    def _build_yolo(self):
        """
        Build the current Luxonis YOLOv6 Nano detection network.
        
        The model is obtained from the Luxonis Model Zoo.
        No local .blob file is required.
        """

        # Describe the model.
        #
        # DepthAI will obtain the appropriate model archive
        # from the Model Zoo.
        model_description = dai.NNModelDescription("yolov6-nano")
       
        # Create detection network
        self.nn = self.pipeline.create(dai.node.DetectionNetwork).build(self.cam, model_description, fps = 15)
       
        # Minimum confidence required for a detection
        self.nn.setConfidenceThreshold(0.5)
       
        # Prevent the neural-network input queue from blocking
        # the camera pipeline if inference temporarily falls behind.
        self.nn.input.setBlocking(False)
       
    # Stereo depth processing
    def _build_stereo_depth(self):
        """Build a stereo-depth pipeline using the OAK-D Lite mono cameras.

        The OAK-D Lite uses CAM_B and CAM_C for its left and right mono sensors.
        The 640x400 inputs match the sensors' 400p mode and limit data transfer.
        """

        self.mono_left = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B
        )
        self.mono_right = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C
        )

        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        left_output = self.mono_left.requestOutput((640, 400), type = dai.ImgFrame.Type.GRAY8, fps = 15)
        right_output = self.mono_right.requestOutput((640, 400),type = dai.ImgFrame.Type.GRAY8, fps = 15)

        left_output.link(self.stereo.left)
        right_output.link(self.stereo.right)

        # Rectify the stereo pair and improve handling of occluded regions.
        self.stereo.setRectification(True)
        self.stereo.setLeftRightCheck(True)
        
##        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
##        self.stereo.setOutputSize(512, 288)
    
    # Helper methods  
    def get_rgb_frame(self):
        """
        Retrieve the newest available RGB frame.

        Older queued frames are discarded to help reduce
        latency in the live camera stream.

        Returns:
            numpy.ndarray or None
        """

        latest_frame = self.q_rgb.tryGet()

        while latest_frame is not None:
            next_frame = self.q_rgb.tryGet()

            if next_frame is None:
                return latest_frame.getCvFrame()

            latest_frame = next_frame

        return None

    def get_depth_frame(self):
        """Retrieve the newest raw stereo depth frame.

        Returns a 2D NumPy array of depth values in millimeters, or None
        when no new depth frame is available. The colorized display image
        will be created later in processors.py.
        """

        latest_depth = self.q_depth.tryGet()

        while latest_depth is not None:
            next_depth = self.q_depth.tryGet()

            if next_depth is None:
                return latest_depth.getFrame()

            latest_depth = next_depth

        return None
    
    def get_detections(self):
        """
        Get the newest detection packet.
        
        Returns:
            dai.ImgDetections or None
        """
        return self.q_nn.tryGet()
    
    def get_label_map(self):
        """
        Get the class names supplied by the model.
        
        Returns:
            list[str] or None
        """

        return self.nn.getClasses()
    
    def is_running(self):
        """
        Return whether the DepthAI pipeline is running or not
        """
        
        return self.pipeline.isRunning()
        
    def close(self):
        """
        Stop the DepthAI pipeline
        """
        
        try:
            self.pipeline.stop()
        except Exception:
            pass
