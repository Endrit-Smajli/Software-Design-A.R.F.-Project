import cv2

class YoloDrawer:
    def draw(self, frame, detections):
        for det in detections:
            h, w = frame.shape[:2]
            x1 = int(det.xmin * w)
            y1 = int(det.ymin * h)
            x2 = int(det.xmax * w)
            y2 = int(det.ymax * h)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{det.label} {det.confidence:.2f}",
                        (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0,255,0), 2)
        return frame


class SimpleSortTracker:
    def __init__(self):
        self.next_id = 1

    def update(self, detections, frame_shape):
        tracks = []
        h, w = frame_shape[:2]
        for det in detections:
            x1 = int(det.xmin * w)
            y1 = int(det.ymin * h)
            x2 = int(det.xmax * w)
            y2 = int(det.ymax * h)
            tracks.append({
                "id": self.next_id,
                "label": det.label,
                "conf": float(det.confidence),
                "x1": x1, "y1": y1, "x2": x2, "y2": y2
            })
            self.next_id += 1
        return tracks

    def draw_tracks(self, frame, tracks):
        for t in tracks:
            cv2.rectangle(frame, (t["x1"], t["y1"]), (t["x2"], t["y2"]), (0,0,255), 2)
            cv2.putText(frame, f'ID {t["id"]}',
                        (t["x1"], t["y1"]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        return frame