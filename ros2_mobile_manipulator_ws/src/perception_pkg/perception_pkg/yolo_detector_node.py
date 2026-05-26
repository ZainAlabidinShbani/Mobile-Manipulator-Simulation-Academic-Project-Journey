"""yolo_detector_node.py

Phase 3 — Perception
Subscribes to /camera/image_raw, runs YOLOv8n inference, and publishes:
  - /perception/detections       (mobile_manipulator_msgs/DetectionArray)
  - /perception/image_annotated  (sensor_msgs/Image)  — bounding-box overlay
  - /bounding_boxes              (mobile_manipulator_msgs/BoundingBoxArray)
  - /detection_confidences       (mobile_manipulator_msgs/DetectionConfidenceArray)

Parameters
----------
model_path         : str   path to YOLOv8 .pt / .onnx model
                           default = 'yolov8n.pt' (auto-downloaded on first run)
confidence_threshold: float  minimum confidence to publish  (default 0.40)
target_classes     : list[str]  COCO class names to keep, empty = all classes
                           default = ['bottle','cup','bowl','box','book','can']
publish_annotated  : bool  publish annotated image         (default True)
use_sim_time       : bool  use /clock for headers           (default True)

Designed for:
  Simulation  — full YOLOv8n PyTorch (.pt)
  Raspberry Pi 4 deployment — swap model_path to yolov8n.onnx
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mobile_manipulator_msgs.msg import (
    BoundingBox,
    BoundingBoxArray,
    Detection,
    DetectionArray,
    DetectionConfidenceArray,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# COCO classes that make sense as lightweight pick-and-place targets
DEFAULT_TARGET_CLASSES: List[str] = [
    "bottle", "cup", "bowl", "book", "can", "box"
]

# Colour palette per class (BGR) for annotation overlay
_PALETTE = [
    (0, 255, 0), (255, 0, 0), (0, 0, 255),
    (255, 255, 0), (0, 255, 255), (255, 0, 255),
]


class YoloDetectorNode(Node):
    """ROS 2 node — YOLOv8n object detector for pick-and-place perception."""

    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter("model_path",          "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.40)
        self.declare_parameter("target_classes",       DEFAULT_TARGET_CLASSES)
        self.declare_parameter("publish_annotated",    True)

        model_path   = self.get_parameter("model_path").value
        self._conf   = float(self.get_parameter("confidence_threshold").value)
        self._target = list(self.get_parameter("target_classes").value)
        self._pub_ann = bool(self.get_parameter("publish_annotated").value)

        # ── load model ────────────────────────────────────────────────
        self._model = None
        self._model_lock = threading.Lock()
        if YOLO_AVAILABLE:
            self.get_logger().info(f"Loading YOLO model: {model_path}")
            try:
                self._model = YOLO(model_path)
                self._class_names: dict = self._model.names  # {id: name}
                self.get_logger().info(
                    f"YOLOv8 loaded — {len(self._class_names)} classes"
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to load YOLO model: {exc}")
        else:
            self.get_logger().warn(
                "ultralytics not installed — node will publish empty detections. "
                "Run: pip install ultralytics"
            )
            self._class_names = {}

        # ── CV bridge ─────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── QoS ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # ── publishers ────────────────────────────────────────────────
        self._pub_detections = self.create_publisher(
            DetectionArray, "/perception/detections", 10
        )
        self._pub_boxes = self.create_publisher(
            BoundingBoxArray, "/bounding_boxes", 10
        )
        self._pub_conf = self.create_publisher(
            DetectionConfidenceArray, "/detection_confidences", 10
        )
        if self._pub_ann:
            self._pub_image = self.create_publisher(
                Image, "/perception/image_annotated", 5
            )

        # ── subscriber ────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_callback,
            sensor_qos,
        )

        self.get_logger().info(
            f"YoloDetectorNode ready | conf≥{self._conf} | "
            f"classes={self._target or 'ALL'}"
        )

    # ─────────────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image) -> None:
        """Main callback — convert ROS image → run YOLO → publish results."""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        det_array   = DetectionArray()
        bbox_array  = BoundingBoxArray()
        conf_array  = DetectionConfidenceArray()
        det_array.header  = msg.header
        bbox_array.header = msg.header
        conf_array.header = msg.header

        detections: List[Detection] = []

        if self._model is not None:
            with self._model_lock:
                results = self._model.predict(
                    cv_img,
                    conf=self._conf,
                    verbose=False,
                )

            for result in results:
                for box in result.boxes:
                    cls_id   = int(box.cls[0])
                    cls_name = self._class_names.get(cls_id, str(cls_id))
                    conf_val = float(box.conf[0])

                    # filter by target classes if specified
                    if self._target and cls_name not in self._target:
                        continue

                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    bbox = BoundingBox(
                        xmin=float(x1), ymin=float(y1),
                        xmax=float(x2), ymax=float(y2),
                    )

                    # placeholder grasp pose — depth_localizer_node refines this
                    grasp = PoseStamped()
                    grasp.header         = msg.header
                    grasp.header.frame_id = "camera_optical_frame"
                    grasp.pose.position.x = (x1 + x2) / 2.0 / msg.width
                    grasp.pose.position.y = (y1 + y2) / 2.0 / msg.height
                    grasp.pose.position.z = 0.0   # filled by depth_localizer
                    grasp.pose.orientation.w = 1.0

                    det = Detection(
                        header     = msg.header,
                        class_id   = cls_name,
                        confidence = conf_val,
                        bbox       = bbox,
                        grasp_pose = grasp,
                    )
                    detections.append(det)

                    # annotate image
                    if self._pub_ann:
                        colour = _PALETTE[cls_id % len(_PALETTE)]
                        cv2.rectangle(
                            cv_img,
                            (int(x1), int(y1)), (int(x2), int(y2)),
                            colour, 2,
                        )
                        label = f"{cls_name} {conf_val:.2f}"
                        cv2.putText(
                            cv_img, label,
                            (int(x1), max(int(y1) - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2,
                        )

        det_array.detections     = detections
        bbox_array.boxes         = [d.bbox       for d in detections]
        conf_array.class_ids     = [d.class_id   for d in detections]
        conf_array.confidences   = [d.confidence for d in detections]

        self._pub_detections.publish(det_array)
        self._pub_boxes.publish(bbox_array)
        self._pub_conf.publish(conf_array)

        if self._pub_ann:
            self._pub_image.publish(
                self._bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
