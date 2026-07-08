"""
perception/detector.py
------------------------
Object detection module. Primary backend uses YOLO (`ultralytics`) on the
camera image produced by the simulation. If `ultralytics`/torch are not
installed, or the model fails to load, a lightweight HSV color-blob detector
is used as a dummy fallback so the perception pipeline always returns
plausible detections (bounding box + estimated bearing/range).

Both backends implement the same `Detection` output format so the rest of
the application (task manager, GUI) is agnostic to which one is active.
"""

from dataclasses import dataclass
import numpy as np
import cv2

import config

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except Exception:  # pragma: no cover
    ULTRALYTICS_AVAILABLE = False


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox: tuple          # (x1, y1, x2, y2) pixel coordinates
    bearing_rad: float    # estimated horizontal bearing relative to camera center
    est_range_m: float    # rough range estimate from apparent size (heuristic)


class YoloDetector:
    def __init__(self, model_path: str = None):
        self.model = YOLO(model_path or config.YOLO_MODEL_PATH)

    def detect(self, image: np.ndarray):
        results = self.model.predict(image, verbose=False, conf=config.YOLO_CONF_THRESHOLD)
        detections = []
        h, w = image.shape[:2]
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                name = self.model.names.get(cls_id, str(cls_id))
                conf = float(box.conf[0])
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                detections.append(_bbox_to_detection(name, conf, (x1, y1, x2, y2), w, h))
        return detections


class ColorBlobDetector:
    """Dummy fallback detector: finds the largest reddish blob in the image
    and reports it as a 'cube' detection. Used when YOLO is unavailable, and
    is also useful for fast unit testing without GPU/model downloads."""

    def __init__(self, target_color_name: str = "cube"):
        self.target_color_name = target_color_name
        self.lower_hsv = np.array([0, 120, 70])
        self.upper_hsv = np.array([10, 255, 255])

    def detect(self, image: np.ndarray):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 20:
            return []
        x, y, bw, bh = cv2.boundingRect(largest)
        h, w = image.shape[:2]
        det = _bbox_to_detection(self.target_color_name, 0.99, (x, y, x + bw, y + bh), w, h)
        return [det]


def _bbox_to_detection(name, conf, bbox, img_w, img_h) -> Detection:
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2.0
    bearing = ((cx - img_w / 2.0) / (img_w / 2.0)) * (config.CAMERA_FOV / 2.0) * (np.pi / 180.0)
    apparent_size = max(x2 - x1, 1.0)
    est_range = float(np.clip(15.0 / apparent_size, 0.15, 5.0))  # heuristic only
    return Detection(class_name=name, confidence=conf, bbox=bbox,
                      bearing_rad=bearing, est_range_m=est_range)


class ObjectDetector:
    """Top-level wrapper that picks YOLO if available, otherwise the dummy
    color-blob detector, and exposes a single `.detect(image)` method."""

    def __init__(self):
        self.backend_name = "dummy_color_blob"
        self._backend = ColorBlobDetector()
        if ULTRALYTICS_AVAILABLE:
            try:
                self._backend = YoloDetector()
                self.backend_name = "yolo"
            except Exception as exc:  # pragma: no cover
                print(f"[detector] Could not load YOLO model ({exc}); using dummy detector.")

    def detect(self, image: np.ndarray):
        if image is None:
            return []
        return self._backend.detect(image)

    def best_target(self, image: np.ndarray):
        """Return the highest-confidence detection matching the accepted
        target classes (or any detection, for the dummy backend)."""
        detections = self.detect(image)
        if not detections:
            return None
        if self.backend_name == "yolo":
            candidates = [d for d in detections if d.class_name in config.TARGET_CLASS_NAMES]
            candidates = candidates or detections
        else:
            candidates = detections
        return max(candidates, key=lambda d: d.confidence)
