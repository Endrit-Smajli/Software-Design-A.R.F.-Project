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

        self.pipeline.start()  # Start the pipeline. This initializes the nodes and prepares them for execution.

        # Create output queues
        self.q_rgb = self.nn.passthrough.createOutputQueue(maxSize = 4, blocking = False)
        self.q_nn = self.nn.out.createOutputQueue(maxSize = 4, blocking = False)

    # ColorCamera node to capture RGB frames from OAK-D Lite sesor.
    def _build_color_camera(self):
        """
        Build the OAK-D Lite RGB camera.
        
        YOLOv6 Nano uses a 512x288 input in the current Luxonis DetectionNetwork example.
        """
        
        self.cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        # Request BGR frames at the model input resolution.
        self.cam_output = self.cam.requestOutput((512, 288), type = dai.ImgFrame.Type.BGR888p, fps = 30)

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
        self.nn = self.pipeline.create(dai.node.DetectionNetwork).build(self.cam_output, model_description)
       
        # Minimum confidence required for a detection
        self.nn.setConfidenceThreshold(0.5)
       
        # Prevent the neural-network input queue from blocking
        # the camera pipeline if inference temporarily falls behind.
        self.nn.input.setBlocking(False)
       
    # Helper methods
    def get_rgb_frame(self):
        """
        Get the newest RGB frame.
        Returns:
            numpy.ndarray or None
        """
        
        frame = self.q_rgb.tryGet()
        if frame is None:
            return None
        
        return frame.getCvFrame()

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
