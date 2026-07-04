"""depth_localizer_node.py

Phase 3 — Perception
Refines the 2D YOLO detections into full 3D grasp poses using the
depth image and camera intrinsics, then transforms each pose from
camera_optical_frame → base_footprint using TF2.

Subscribes
----------
/perception/detections         mobile_manipulator_msgs/DetectionArray
/camera/depth/image_raw        sensor_msgs/Image  (32FC1 metres)
/camera/camera_info            sensor_msgs/CameraInfo

Publishes
---------
/perception/target_pose        geometry_msgs/PoseStamped  (best target)
/perception/target_markers     visualization_msgs/MarkerArray  (RViz spheres)
"""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mobile_manipulator_msgs.msg import DetectionArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

import tf2_geometry_msgs  # registers PoseStamped transform support


class DepthLocalizerNode(Node):
    """Projects YOLO bounding-box centres into 3-D using depth + camera K."""

    def __init__(self) -> None:
        super().__init__("depth_localizer_node")

        self.declare_parameter("target_frame",   "base_footprint")
        self.declare_parameter("depth_scale",    1.0)   # Gazebo depth in metres
        self.declare_parameter("min_depth",      0.10)
        self.declare_parameter("max_depth",      5.00)
        self.declare_parameter("publish_markers", True)

        self._target_frame   = self.get_parameter("target_frame").value
        self._depth_scale    = float(self.get_parameter("depth_scale").value)
        self._min_d          = float(self.get_parameter("min_depth").value)
        self._max_d          = float(self.get_parameter("max_depth").value)
        self._pub_markers    = bool(self.get_parameter("publish_markers").value)

        # camera intrinsics (filled on first CameraInfo message)
        self._K: np.ndarray | None = None

        self._bridge = CvBridge()
        self._latest_depth: np.ndarray | None = None
        self._latest_depth_header = None

        # TF2
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Subscribers
        self.create_subscription(
            CameraInfo, "/camera/camera_info",
            self._camera_info_cb, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw",
            self._depth_cb, sensor_qos
        )
        self.create_subscription(
            DetectionArray, "/detections",
            self._detection_cb, 10
        )

        # Publishers
        self._pub_pose = self.create_publisher(
            PoseStamped, "/perception/target_pose", 10
        )
        if self._pub_markers:
            self._pub_markers_pub = self.create_publisher(
                MarkerArray, "/perception/target_markers", 10
            )

        self.get_logger().info(
            f"DepthLocalizerNode ready — target_frame={self._target_frame}"
        )

    # ─────────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._K is None:
            self._K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"Camera intrinsics received: fx={self._K[0,0]:.1f} "
                f"fy={self._K[1,1]:.1f} cx={self._K[0,2]:.1f} cy={self._K[1,2]:.1f}"
            )

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="32FC1"
            )
            self._latest_depth_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")

    def _detection_cb(self, msg: DetectionArray) -> None:
        """For each detection, sample depth at bbox centre and project to 3-D."""
        if self._K is None or self._latest_depth is None:
            return
        if not msg.detections:
            return

        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]

        marker_array = MarkerArray()
        best_pose: PoseStamped | None = None
        best_z = float("inf")

        for idx, det in enumerate(msg.detections):
            # pixel centre of bounding box
            u = (det.bbox.xmin + det.bbox.xmax) / 2.0
            v = (det.bbox.ymin + det.bbox.ymax) / 2.0

            # sample depth with a small 3×3 median window for robustness
            h, w = self._latest_depth.shape
            u_i, v_i = int(np.clip(u, 0, w - 1)), int(np.clip(v, 0, h - 1))
            roi = self._latest_depth[
                max(0, v_i - 1):min(h, v_i + 2),
                max(0, u_i - 1):min(w, u_i + 2),
            ]
            valid = roi[(roi > self._min_d) & (roi < self._max_d)]
            if valid.size == 0:
                continue
            depth = float(np.median(valid)) * self._depth_scale

            # back-project to camera frame
            x_cam = (u - cx) * depth / fx
            y_cam = (v - cy) * depth / fy
            z_cam = depth

            # build PoseStamped in camera_optical_frame
            pose_cam = PoseStamped()
            pose_cam.header.stamp    = self._latest_depth_header.stamp
            pose_cam.header.frame_id = "camera_optical_frame"
            pose_cam.pose.position.x = x_cam
            pose_cam.pose.position.y = y_cam
            pose_cam.pose.position.z = z_cam
            pose_cam.pose.orientation.w = 1.0

            # transform to target frame
            try:
                pose_world = self._tf_buffer.transform(
                    pose_cam,
                    self._target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"TF transform failed [{det.class_id}]: {exc}"
                )
                continue

            # pick closest object as primary target
            if depth < best_z:
                best_z    = depth
                best_pose = pose_world

            # build RViz marker
            if self._pub_markers:
                m = Marker()
                m.header         = pose_world.header
                m.ns             = "detections"
                m.id             = idx
                m.type           = Marker.SPHERE
                m.action         = Marker.ADD
                m.pose           = pose_world.pose
                m.scale.x        = 0.08
                m.scale.y        = 0.08
                m.scale.z        = 0.08
                m.color.r        = 0.0
                m.color.g        = 1.0
                m.color.b        = 0.4
                m.color.a        = 0.85
                m.lifetime.sec   = 1
                marker_array.markers.append(m)

        if best_pose is not None:
            self._pub_pose.publish(best_pose)

        if self._pub_markers and marker_array.markers:
            self._pub_markers_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
