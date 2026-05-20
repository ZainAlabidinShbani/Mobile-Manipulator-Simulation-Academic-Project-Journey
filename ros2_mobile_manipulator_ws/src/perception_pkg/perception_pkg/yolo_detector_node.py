import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mobile_manipulator_msgs.msg import BoundingBox, BoundingBoxArray, Detection, DetectionArray, DetectionConfidenceArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.declare_parameter('target_class', 'cube')
        self.declare_parameter('mock_detection', True)
        self.declare_parameter('confidence', 0.92)
        self.declare_parameter('bbox_size_px', 84)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.camera_info = None
        self.frame_index = 0
        self.detect_pub = self.create_publisher(DetectionArray, '/detections', 10)
        self.box_pub = self.create_publisher(BoundingBoxArray, '/bounding_boxes', 10)
        self.conf_pub = self.create_publisher(DetectionConfidenceArray, '/detection_confidences', 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 10)

    def _info_cb(self, msg):
        self.camera_info = msg

    def _image_cb(self, msg):
        self.frame_index += 1
        if not bool(self.get_parameter('mock_detection').value):
            return
        width = msg.width or 640
        height = msg.height or 480
        cx = width * 0.5 + math.sin(self.frame_index * 0.2) * (width * 0.08)
        cy = height * 0.52 + math.cos(self.frame_index * 0.17) * (height * 0.05)
        size = int(self.get_parameter('bbox_size_px').value)
        half = size / 2.0
        bbox = BoundingBox()
        bbox.xmin = max(0.0, cx - half)
        bbox.xmax = min(float(width), cx + half)
        bbox.ymin = max(0.0, cy - half)
        bbox.ymax = min(float(height), cy + half)

        det = Detection()
        det.header = msg.header
        det.class_id = str(self.get_parameter('target_class').value)
        det.confidence = float(self.get_parameter('confidence').value)
        det.bbox = bbox
        grasp = PoseStamped()
        grasp.header = msg.header
        grasp.header.frame_id = 'base_link'
        grasp.pose.position.x = 1.0 + math.sin(self.frame_index * 0.1) * 0.1
        grasp.pose.position.y = math.cos(self.frame_index * 0.1) * 0.05
        grasp.pose.position.z = 0.82
        grasp.pose.orientation.w = 1.0
        det.grasp_pose = grasp

        detections = DetectionArray()
        detections.header = msg.header
        detections.detections = [det]

        boxes = BoundingBoxArray()
        boxes.header = msg.header
        boxes.boxes = [bbox]

        confidences = DetectionConfidenceArray()
        confidences.header = msg.header
        confidences.class_ids = [det.class_id]
        confidences.confidences = [det.confidence]

        self.detect_pub.publish(detections)
        self.box_pub.publish(boxes)
        self.conf_pub.publish(confidences)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
