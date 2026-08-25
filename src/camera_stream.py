"""
camera_stream.py

Handles USB webcam capture on a Raspberry Pi and makes frames available:
  1. Locally, to feed into your existing vision-processing code.
  2. Remotely, via a Flask MJPEG stream you can view in any browser
     (e.g. http://<pi-ip>:5000/video_feed) from your laptop/phone.

Both consumers share a single camera thread, so the USB webcam is only
opened once, avoiding bandwidth/lock issues on the Pi.

Install deps on the Pi:
    pip install opencv-python flask --break-system-packages
    (or use a venv: python3 -m venv venv && source venv/bin/activate && pip install opencv-python flask)
"""

import cv2
import threading
import time
from flask import Flask, Response


class VideoStream:
    """
    Continuously reads frames from a USB webcam in a background thread
    and stores the latest frame. Any number of readers (your vision code,
    the streaming server) can grab the most recent frame without blocking
    each other or re-reading the camera.
    """

    def __init__(self, src=0, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera source {src}. "
                f"Check it's plugged in and try `ls /dev/video*` to confirm the device index."
            )

        self.lock = threading.Lock()
        self.frame = None
        self.running = False

        # Prime the buffer with one frame before starting threads elsewhere
        ok, frame = self.cap.read()
        if ok:
            self.frame = frame

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = frame
            else:
                # Camera hiccup - brief pause before retrying instead of busy-looping
                time.sleep(0.05)

    def read(self):
        """Returns the most recent frame (thread-safe copy)."""
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        self.cap.release()


# ---------------------------------------------------------------------------
# Remote viewing: MJPEG stream over HTTP
# ---------------------------------------------------------------------------

app = Flask(__name__)
video_stream = None  # set in main()


def generate_mjpeg():
    while True:
        frame = video_stream.read()
        if frame is None:
            time.sleep(0.01)
            continue

        ok, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return '<html><body><h2>Robot Dog Camera</h2><img src="/video_feed"></body></html>'


# ---------------------------------------------------------------------------
# Local processing: plug in your existing vision code here
# ---------------------------------------------------------------------------

def process_frame_with_vision(frame):
    """
    Replace this with a call into your existing vision module, e.g.:
        detections = my_vision_module.detect(frame)
        return detections
    Keep this function fast - it runs in the main loop alongside movement
    control, so avoid heavy blocking calls here if you can help it.
    """
    # Placeholder: just return the frame unchanged.
    return frame


def robot_control_loop():
    """
    Runs in its own thread: continuously pulls the latest frame, runs your
    vision code on it, and is where you'd plug in movement decisions
    (e.g. call your motor control functions based on detection results).
    """
    while True:
        frame = video_stream.read()
        if frame is None:
            time.sleep(0.01)
            continue

        result = process_frame_with_vision(frame)

        # Example hook point:
        # if result.obstacle_detected:
        #     robot_motors.stop()
        # else:
        #     robot_motors.walk_forward()

        time.sleep(0.03)  # ~30 fps loop; tune to match your vision code's speed


def main():
    global video_stream

    # src=0 is usually correct for a single USB webcam; if you have multiple
    # cameras or it's not detected, check `ls /dev/video*` and try that index.
    video_stream = VideoStream(src=0, width=640, height=480, fps=30).start()

    # Give the camera a moment to warm up
    time.sleep(1.0)

    # Start local vision/control processing in the background
    control_thread = threading.Thread(target=robot_control_loop, daemon=True)
    control_thread.start()

    # Run the remote-viewing web server in the main thread.
    # Visit http://<raspberry-pi-ip>:5000/ from your laptop/phone on the same network.
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        video_stream.stop()


if __name__ == '__main__':
    main()
