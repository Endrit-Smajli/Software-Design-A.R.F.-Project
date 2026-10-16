# NAVIGATION FOR THE ROBOT WHERE THE LIDAR SCANS THE ROOM FOR OBJECTS 
import cv2
import numpy as np
from path_planning import plan_path

path = plan_path(occupancy_grid, start, goal)

if path is None:
    print("No safe route found")
else:
    print("Planned route:", path)

class RoomNavigator:
    def __init__(self):
        self.last_edges = None

    def analyze_frame(self, frame):
        """
        Analyze a camera frame for visual edges.

        Returns:
            edges: binary edge image
            overlay: original frame with edges highlighted

        Note: edges alone do not identify obstacle distance
        or prove that a region is safe to walk through.
        """
        if frame is None:
            return None, None

        # 1. Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Reduce camera noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3. Compute image gradients (spatial intensity changes)
        grad_x = cv2.Sobel(
            blurred, cv2.CV_32F, 1, 0, ksize=3
        )
        grad_y = cv2.Sobel(
            blurred, cv2.CV_32F, 0, 1, ksize=3
        )

        # Gradient magnitude: sqrt(Gx^2 + Gy^2)
        magnitude = np.sqrt(grad_x**2 + grad_y**2)

        # 4. Detect edges
        edges = cv2.Canny(blurred, 60, 150)
        self.last_edges = edges

        # 5. Highlight edges on the original frame
        overlay = frame.copy()
        overlay[edges > 0] = (0, 255, 0)

        # 6. Analyze edge activity in the lower image region.
        # This is a visual diagnostic, NOT a reliable free-space map.
        height, width = edges.shape
        floor_region = edges[int(height * 0.55):, :]

        left = floor_region[:, :width // 3]
        center = floor_region[:, width // 3:2 * width // 3]
        right = floor_region[:, 2 * width // 3:]

        edge_activity = {
            "left": float(np.mean(left > 0)),
            "center": float(np.mean(center > 0)),
            "right": float(np.mean(right > 0)),
        }

        return edges, overlay, edge_activity


if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    navigator = RoomNavigator()

    if not camera.isOpened():
        raise RuntimeError("Could not open camera")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            edges, overlay, activity = navigator.analyze_frame(frame)

            print("Visual edge activity:", activity)

            cv2.imshow("Room edges", edges)
            cv2.imshow("Camera overlay", overlay)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
