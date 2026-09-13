import depthai as dai   # DetphAi SDK import

class OakDDevice:
    
    # Establish DetphAI pipeline
    def __init__(self):
        self.pipeline = dai.Pipeline()

        self._build_color_camera()  # Build color camera node in the pipeline
        self._build_yolo()          # Build YOLO node in the pipeline

        # Upload pipeline to OAK-D Lite and start the device.
        self.device = dai.Device(self.pipeline)
        
        # Create output queues to receive data from the device. 
        # `maxSize` sets the maximum number of frames that can be stored in the queue. 
        # `blocking = False` means that if the queue is full, the oldest frame will be dropped to make room for the new frame.
        # (.get() wil not freeze the program.)
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)   # Raw camera frames
        self.q_nn  = self.device.getOutputQueue("nn",  maxSize=4, blocking=False)   # YOLO detections

    # ColorCamera node to capture RGB frames from OAK-D Lite sesor.
    def _build_color_camera(self):
        cam = self.pipeline.createColorCamera()                                     # Create ColorCamera node in the pipeline.
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)    # Set camera resolution.
        cam.setPreviewSize(640, 360)                                                # Set preview size.     
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)                 # Set color order to BGR (OpenCV format, to avoid color space conversion).
        cam.setFps(30)                                                              # Set camera FPS.

        xout_rgb = self.pipeline.createXLinkOut()   # Create XLinkOut node for RGB output.
        xout_rgb.setStreamName("rgb")               # Set stream name for RGB output. /rgb
        cam.preview.link(xout_rgb.input)            # Link ColorCamera preview output to XLinkOut input.

        self.cam = cam  # Store the ColorCamera node in the class instance for later use. (For future camera settings adjustments.)

    # YOLO node to perform object detection on the RGB frames.
    def _build_yolo(self):
        nn = self.pipeline.createYoloDetectionNetwork() # Create YOLO detection network node in the pipeline.
        nn.setConfidenceThreshold(0.5)                  # Minimum confidence threshold for detections. Detections with confidence below this value will be ignored.
        nn.setNumClasses(80)                            # YOLOv5 model has 80 classes (COCO dataset).
        nn.setCoordinateSize(4)                         # Set the size of the bounding box coordinates (x, y, width, height).
        
        # Set YOLO anchors and anchor masks:
        
        # Anchors are predefined bounding boxes that the model uses to predict object locations.
        # Must match blob file.
        nn.setAnchors([
            10,13, 16,30, 33,23,
            30,61, 62,45, 59,119,
            116,90, 156,198, 373,326
        ])
        
        # Anchor masks define which anchors are used for each detection layer.
        # Must match the blob file.
        nn.setAnchorMasks({
            "side26": [0,1,2],
            "side13": [3,4,5]
        })
        
        # Set Intersection over Union (IoU) threshold for non-max suppression. 
        # Detections with IoU above this threshold will be suppressed.
        nn.setIouThreshold(0.5)
        
        # Load the YOLOv5 model blob file. 
        # The blob file is a compiled version of the model that can be run on the OAK-D Lite device.
        nn.setBlobPath("yolov5n_openvino_2021.4_6shave.blob")

        # Link the ColorCamera preview output to the YOLO input. 
        # This means that the YOLO node will receive the RGB frames from the camera for object detection.
        self.cam.preview.link(nn.input)

        
        xout_nn = self.pipeline.createXLinkOut()    # Create XLinkOut node for YOLO.
        xout_nn.setStreamName("nn")                 # Read detections from this stream. /nn
        nn.out.link(xout_nn.input)                  # Link YOLO output to XLinkOut input.

        self.nn = nn    # Store the YOLO node in the class instance for later use. (For future YOLO settings adjustments.)
