import depthai as dai

class OakDDevice:
    def __init__(self):
        self.pipeline = dai.Pipeline()

        self._build_color_camera()
        self._build_yolo()

        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.q_nn  = self.device.getOutputQueue("nn",  maxSize=4, blocking=False)

    def _build_color_camera(self):
        cam = self.pipeline.createColorCamera()
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setPreviewSize(640, 360)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(30)

        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        cam.preview.link(xout_rgb.input)

        self.cam = cam

    def _build_yolo(self):
        nn = self.pipeline.createYoloDetectionNetwork()
        nn.setConfidenceThreshold(0.5)
        nn.setNumClasses(80)
        nn.setCoordinateSize(4)
        nn.setAnchors([
            10,13, 16,30, 33,23,
            30,61, 62,45, 59,119,
            116,90, 156,198, 373,326
        ])
        nn.setAnchorMasks({
            "side26": [0,1,2],
            "side13": [3,4,5]
        })
        nn.setIouThreshold(0.5)
        nn.setBlobPath("yolov5n_openvino_2021.4_6shave.blob")

        self.cam.preview.link(nn.input)

        xout_nn = self.pipeline.createXLinkOut()
        xout_nn.setStreamName("nn")
        nn.out.link(xout_nn.input)

        self.nn = nn