import cv2  

# Helper class to draw bounding boxes and labels on frames.
class YoloDrawer:
    
    # Draws bounding boxes and labels on the given frame based on the provided detections.
    # frame: The image from the camera (BGR, h x w x 3).
    # detections: List of detection objects containing bounding box coordinates and labels.
    def draw(self, frame, detections):
        
        # Draw bounding boxes and labels on the frame for each detection.
        for det in detections:
            
            h, w = frame.shape[:2]  # Get the height and width of the frame. (frame.shape is h, w, channels)
            
            # Convert normalized bounding box coordinates to pixel coordinates:
            # det.xmin, det.ymin, det.xmax, det.ymax are in [0, 1] range, 
            # so we multiply by width and height to get pixel values.
            # Cast to int for OpenCV drawing functions integer coordinates.
            x1 = int(det.xmin * w)  
            y1 = int(det.ymin * h)
            x2 = int(det.xmax * w)
            y2 = int(det.ymax * h)

            # Draw a green rectangle around the detected object.
            # (x1, y1): top-left corner
            # (x2, y2): bottom-right corner
            # (0,255,0): color in BGR
            # 2: thickness.
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            
            # Draw the label and confidence score above the bounding box.
            # Text format: "label confidence"
            # (x1, y1-10): position of the text (slightly above the bounding box)
            # cv2.FONT_HERSHEY_SIMPLEX: font type
            # 0.5: font scale
            # (0,255,0): green color in BGR
            # 2: thickness of the text.
            cv2.putText(frame, f"{det.label} {det.confidence:.2f}",
                        (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0,255,0), 2)
        
        return frame    # Return modified frame with bounding boxes and labels drawn.

# Simple tracking class (placeholder that assigns incremental IDs to detections per frame).
class SimpleSortTracker:
    
    # Initializes the tracker with a starting ID for detections.
    def __init__(self):
        self.next_id = 1

    # Update tracking state based on current detections.
    # - detections: YOLO detections for the current frame.
    # - frame_shape: Shape of the current frame (height, width, channels), 
    #                to convert normalized coordinates to pixel coordinates.
    def update(self, detections, frame_shape):
        
        # Initialize empty list to hold tracking information for the current frame.
        tracks = []
        # Get height and width of the frame from its shape. 
        # Used for coordinate conversion.
        h, w = frame_shape[:2]
        
        # Create one "track" for each detection.
        for det in detections:
            
            # normalized YOLO box to pixel coordinates conversion:
            x1 = int(det.xmin * w)
            y1 = int(det.ymin * h)
            x2 = int(det.xmax * w)
            y2 = int(det.ymax * h)
            
            # Append a dictionary representing the tracked object to the tracks list.
            # - "id": unique ID for the detection (incremental)
            # - "label": class label of the detection
            # - "conf": confidence score of the detection
            # - "x1", "y1", "x2", "y2": pixel coordinates of the bounding box.
            tracks.append({
                "id": self.next_id,
                "label": det.label,
                "conf": float(det.confidence),
                "x1": x1, "y1": y1, "x2": x2, "y2": y2
            })
            
            # ID increment for the next detection in the next frame.
            self.next_id += 1
            
        return tracks   # Return list of tracks.

    # Draw tracking boxes and IDs on the given frame based on the provided tracks.
    # frame: The image from the camera (BGR, h x w x 3).
    # tracks: List of track dicts from `update()`.
    def draw(self, frame, tracks):
        
        # Draw each tracked object
        for t in tracks:
            
            # Draw a red rectangle around the tracked object.
            # (t["x1"], t["y1"]): top-left corner
            # (t["x2"], t["y2"]): bottom-right corner
            # (0,0,255): red color in BGR
            # 2: thickness.
            cv2.rectangle(frame, (t["x1"], t["y1"]), (t["x2"], t["y2"]), (0,0,255), 2)
            
            # Draw tracking ID above box.
            # Text format: "ID {id}"
            # (t["x1"], t["y1"]-10): position of the text (slightly above the bounding box)
            # cv2.FONT_HERSHEY_SIMPLEX: font type
            # (0,0,255): red color in BGR
            # 2: thickness of the text.
            cv2.putText(frame, f'ID {t["id"]}',
                        (t["x1"], t["y1"]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        
        return frame    # Return modified frame.
