import cv2  

# COCO class labels for YOLOv6. 
# These are the 80 object categories that the model can detect.
COCO_LABELS = [ 
    "person", 
    "bicycle", 
    "car", 
    "motorcycle", 
    "airplane", 
    "bus", 
    "train", 
    "truck", 
    "boat", 
    "traffic light", 
    "fire hydrant", 
    "stop sign", 
    "parking meter", 
    "bench", 
    "bird", 
    "cat", 
    "dog", 
    "horse", 
    "sheep", 
    "cow", 
    "elephant", 
    "bear", 
    "zebra", 
    "giraffe", 
    "backpack", 
    "umbrella", 
    "handbag", 
    "tie", 
    "suitcase", 
    "frisbee", 
    "skis", 
    "snowboard", 
    "sports ball", 
    "kite", 
    "baseball bat", 
    "baseball glove", 
    "skateboard", 
    "surfboard", 
    "tennis racket", 
    "bottle", 
    "wine glass", 
    "cup", "fork", 
    "knife", 
    "spoon", 
    "bowl", 
    "banana", 
    "apple", 
    "sandwich", 
    "orange", 
    "broccoli", 
    "carrot", 
    "hot dog", 
    "pizza", 
    "donut", 
    "cake", 
    "chair", 
    "couch", 
    "potted plant", 
    "bed", 
    "dining table", 
    "toilet", 
    "tv", 
    "laptop", 
    "mouse", 
    "remote", 
    "keyboard", 
    "cell phone", 
    "microwave", 
    "oven", 
    "toaster", 
    "sink", 
    "refrigerator", 
    "book", 
    "clock", 
    "vase", 
    "scissors", 
    "teddy bear", 
    "hair drier", 
    "toothbrush", 
]
# Helper class to draw bounding boxes and labels on frames.
class YoloDrawer:
    """
    Draw YOLO detection bounding boxes and labels.
    
    Compatible with dai.ImgDetection objects produced by
    DepthAI v3 DetectionNetwork.
    """
    def __init__(self,label_map = None):
        """
        label_map:
            Optional list of class names
      
        If not provided, COCO_LABELS will be used.
        """
        self.label_map = label_map or COCO_LABELS  # Use provided label map or default to COCO labels.
        
    # Convert class ID to class name
    def get_label(self, label_id):
        """
        Convert a numeric detection label into a readable name
 
        """
        try:
            label_id = int(label_id)
            
            if 0 <= label_id < len(self.label_map):
                return self.label_map[label_id]  # Return the corresponding class name.
        except (TypeError, ValueError):
            pass  # If label_id is not an integer or out of range, return None.
        
        # Fallback if the label is invalid.
        return str(label_id)
    
    # Draws bounding boxes and labels on the given frame based on the provided detections.
    # frame: The image from the camera (BGR, h x w x 3).
    # detections: List of detection objects containing bounding box coordinates and labels.
    def draw(self, frame, detections):
        """
        Draw bounding boxes and labels.
        
        Parameters:
        - frame: OpenCV BGR image
        - detections: List of dai.ImgDetection objects
        """
        
        if frame is None:
            return frame  # If the frame is None, return it as is (no drawing possible).
        
        h, w = frame.shape[:2]  # Get the height and width of the frame. (frame.shape is h, w, channels)
        
        for det in detections:
            # Convert normalized coordinates to pixels
            x1 = int(det.xmin * w)
            y1 = int(det.ymin * h)
            x2 = int(det.xmax * w)
            y2 = int(det.ymax * h)
            
            # keep coordinates inside the image
            x1 = max(0, min(x1, w-1))
            y1 = max(0, min(y1, h-1))
            x2 = max(0, min(x2, w-1))
            y2 = max(0, min(y2, h-1))
            
            # Get label
            label = self.get_label(det.label)
            
            confidence = float(det.confidence)  
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2 )
            
            # Draw label
            text = f"{label} {confidence:.2f}"
            
            text_y = max(y1 - 10, 20)  # Ensure text is not drawn above the image
            
            cv2.putText(frame, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        return frame

# Simple tracking class (placeholder that assigns incremental IDs to detections per frame).
class SimpleSortTracker:
    """
    Simple host-side tracker
    
    NOTE:
    This is still a basic tracker. It assigns a persistent ID 
    only while detections can be matched by the simple center-distance / class matching logic.
    
    It is NOT a full SORT/ByteTrack/DeepSORT implementation.
    """
   
    def __init__(self, max_distance = 80, max_missing = 10):
        """
        max_distance:
            Maximum pixel distance allowed when matching a
            detection to an existing track.
        
        max_missing:
            Number of consecutive frames a track can disappear
            before it is removed.
        """
        self.next_id = 1

        self.max_distance = max_distance
        self.max_missing = max_missing
        
        self.tracks = {}  # Dictionary to hold active tracks. Key: track ID, Value: track info (dict)
    
    # Create a track dictionary from a detection
    def _make_detection(self, det, frame_shape):
        """
        Convert a dai.ImgDetection into a tracking dictionary.
        """
        
        h, w = frame_shape[:2]
        
        x1 = int(det.xmin * w)
        y1 = int(det.ymin * h)
        x2 = int(det.xmax * w)
        y2 = int(det.ymax * h)
        
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(0, min(x2, w - 1))
        y2 = max(0, min(y2, h - 1))
        
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        
        return {
            "label_id": int(det.label),
            "label": self._label_name(det.label),
            "conf": float(det.confidence),
            
            "x1": x1, 
            "y1": y1, 
            "x2": x2, 
            "y2": y2,
            
            "cx": cx,
            "cy": cy
            
        }
        
    # Convert class ID to class name
    def _label_name(self, label_id):
        """
        Convert COCO class ID to class name.
        """
            
        try:
            label_id = int(label_id)
                
            if 0 <= label_id < len(COCO_LABELS):
                return COCO_LABELS[label_id]
        except (TypeError, ValueError):
            pass
            
        return str(label_id)  # Fallback to string representation if label ID is invalid.

    # Calculate center distance
    def _distance(self, track, detection):
        """
        Calculate Euclidean distance between two bounding-box centers.
        """
        dx = track["cx"] - detection["cx"]
        dy = track["cy"] - detection["cy"]
        
        return (dx ** 2 + dy ** 2) ** 0.5  # Return Euclidean distance.
    
    # Update tracker
    def update(self, detections, frame_shape):
        """
        Update tracking information using the current detections.
        
        Returns:
            List of dictionaries containing tracked objects.
        """
        
        # Convert detections into internal format.
        current_detections = [
            self._make_detection(det, frame_shape) 
            for det in detections
            ]
        
        # Mark existing tracks as missing
        for track in self.tracks.values():
            track["missing"] += 1
        
        # Keep track of which tracks have already been matched.
        matched_track_ids = set()
        
        # Match each detection to an existing track
        for detection in current_detections:
            
            best_track_id = None
            best_distance = self.max_distance
            
            for track_id, track in self.tracks.items():
                
                # A track cannot be used twice in the same frame.
                if track_id in matched_track_ids:
                    continue
                
                # Only match the same object class.
                if track["label_id"] != detection["label_id"]:
                    continue
                
                distance = self._distance(track, detection)
                
                if distance < best_distance:
                    best_distance = distance
                    best_track_id = track_id
            
            # Existing track found
            if best_track_id is not None:
                
                track = self.tracks[best_track_id]
                
                track.update(detection) # Update track with new detection info.
                track["missing"] = 0    # Reset missing counter
                
                matched_track_ids.add(best_track_id)  # Mark this track as matched.
                
            # No existing track found
            else:
                
                new_id = self.next_id
                self.next_id += 1
                
                detection["id"] = new_id
                detection["missing"] = 0
                
                self.tracks[new_id] = detection  # Add new track.
                
                matched_track_ids.add(new_id)  # Mark this new track as matched.
        
        # Remove tracks that have been missing for too long.
        remove_ids = []
        
        for track_id, track in self.tracks.items():
            
            if track["missing"] > self.max_missing:
                remove_ids.append(track_id)
        
        for track_id in remove_ids:
            del self.tracks[track_id]  # Remove track from active tracks.
            
        # Return currently visible tracks
        visible_tracks = []
        
        for track in self.tracks.values():
            
            if track["missing"] == 0:
                
                visible_tracks.append({
                    "id": track["id"],
                    "label_id": track["label_id"],
                    "label": track["label"],
                    "conf": track["conf"],
                    
                    "x1": track["x1"],
                    "y1": track["y1"],
                    "x2": track["x2"],
                    "y2": track["y2"],
                })
       
        return visible_tracks  # Return list of currently visible tracks.
    
    # Draw tracking boxes and IDs
    def draw_tracks(self, frame, tracks):
        """
        Draw tracked objects and their persistent IDs.
        """
        
        if frame is None:
            return frame  # If the frame is None, return it as is (no drawing possible).
        
        for track in tracks:
            
            x1 = track["x1"]
            y1 = track["y1"]
            x2 = track["x2"]
            y2 = track["y2"]
            
            track_id = track["id"]
            label = track["label"]
            
            # Draw tracking bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # Draw ID and label
            text = f"ID {track_id}: {label}"
            
            text_y = max(y1 - 10, 20)  # Ensure text is not drawn above the image
            
            cv2.putText(
                frame,
                text,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2
            )
        return frame  # Return modified frame with tracking boxes and IDs.
    
    # Backwards-compatible draw method
    def draw(self, frame, tracks):
        """
        Alias for draw_tracks().
        
        Kept so existing code that calls tracker.draw()
        will continue to work without modification.
        """
        return self.draw_tracks(frame, tracks)