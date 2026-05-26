import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mobile_manipulator_msgs.msg import DetectionArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class GraspPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('grasp_pose_estimator_node')
        self.declare_parameter('object_depth', 0.78)
        self.declare_parameter('camera_offset_x', 0.15)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.48)
        self.declare_parameter('target_frame', 'base_link')
        self.camera_info = None
        self.latest_pose = None
        self.pose_pub = self.create_publisher(PoseStamped, '/grasp_target_pose', 10)
        self.create_subscription(DetectionArray, '/detections', self._detections_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 10)

    def _info_cb(self, msg):
        self.camera_info = msg

    def _detections_cb(self, msg):
        if not msg.detections:
            return
        det = max(msg.detections, key=lambda d: d.confidence)
        if self.camera_info is None:
            return
        info = self.camera_info
        fx = info.k[0] if info.k else 525.0
        fy = info.k[4] if info.k else 525.0
        cx = info.k[2] if info.k else info.width / 2.0
        cy = info.k[5] if info.k else info.height / 2.0
        px = (det.bbox.xmin + det.bbox.xmax) / 2.0
        py = (det.bbox.ymin + det.bbox.ymax) / 2.0
        depth = float(self.get_parameter('object_depth').value)
        x_cam = (px - cx) * depth / fx
        y_cam = (py - cy) * depth / fy
        pose = PoseStamped()
        pose.header = det.header
        pose.header.frame_id = str(self.get_parameter('target_frame').value)
        pose.pose.position.x = float(self.get_parameter('camera_offset_x').value) + depth
        pose.pose.position.y = float(self.get_parameter('camera_offset_y').value) + x_cam
        pose.pose.position.z = float(self.get_parameter('camera_offset_z').value) - 0.25 + y_cam
        pose.pose.orientation.w = 1.0
        self.latest_pose = pose
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = GraspPoseEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
