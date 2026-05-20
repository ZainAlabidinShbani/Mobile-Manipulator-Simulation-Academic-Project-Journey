import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from mobile_manipulator_msgs.msg import DetectionArray, RobotState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster


def _yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RVizBridgeNode(Node):
    def __init__(self):
        super().__init__('rviz_bridge_node')
        self.declare_parameter('camera_x', 0.15)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.48)
        self.declare_parameter('arm_x', 0.0)
        self.declare_parameter('arm_y', 0.0)
        self.declare_parameter('arm_z', 0.32)
        self.detections = None
        self.robot_state = None
        self.robot_pose = None
        self.map = None
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(DetectionArray, '/detections', self._detections_cb, 10)
        self.create_subscription(RobotState, '/robot_state', self._robot_state_cb, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self._robot_pose_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.timer = self.create_timer(0.5, self._publish_static_transforms)
        self.render_timer = self.create_timer(0.25, self._publish_markers)

    def _detections_cb(self, msg):
        self.detections = msg

    def _robot_state_cb(self, msg):
        self.robot_state = msg

    def _robot_pose_cb(self, msg):
        self.robot_pose = msg

    def _map_cb(self, msg):
        self.map = msg

    def _publish_static_transforms(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child, x, y, z in (
            ('camera_link', float(self.get_parameter('camera_x').value), float(self.get_parameter('camera_y').value), float(self.get_parameter('camera_z').value)),
            ('arm_base_link', float(self.get_parameter('arm_x').value), float(self.get_parameter('arm_y').value), float(self.get_parameter('arm_z').value)),
        ):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation = _yaw_to_quaternion(0.0)
            transforms.append(t)
        self.static_tf.sendTransform(transforms)

    def _make_marker(self, marker_id, marker_type, frame_id, x, y, z, scale, color, text=''):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mobile_manipulator'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color = color
        marker.text = text
        return marker

    def _publish_markers(self):
        markers = MarkerArray()
        if self.robot_pose is not None:
            p = self.robot_pose.pose.position
            markers.markers.append(self._make_marker(
                1, Marker.CUBE, self.robot_pose.header.frame_id, p.x, p.y, 0.08,
                (0.45, 0.35, 0.12), ColorRGBA(r=0.2, g=0.6, b=0.9, a=0.8)
            ))
            if self.robot_state is not None:
                markers.markers.append(self._make_marker(
                    2, Marker.TEXT_VIEW_FACING, self.robot_pose.header.frame_id, p.x, p.y, 0.55,
                    (0.0, 0.0, 0.18), ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                    f'{self.robot_state.mode} / {self.robot_state.task_state}'
                ))
        if self.detections is not None and self.detections.detections:
            det = self.detections.detections[0]
            center_x = (det.bbox.xmin + det.bbox.xmax) / 2.0
            center_y = (det.bbox.ymin + det.bbox.ymax) / 2.0
            markers.markers.append(self._make_marker(
                3, Marker.SPHERE, det.grasp_pose.header.frame_id,
                det.grasp_pose.pose.position.x, det.grasp_pose.pose.position.y, det.grasp_pose.pose.position.z,
                (0.08, 0.08, 0.08), ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9),
            ))
            markers.markers.append(self._make_marker(
                4, Marker.TEXT_VIEW_FACING, det.grasp_pose.header.frame_id,
                det.grasp_pose.pose.position.x, det.grasp_pose.pose.position.y, det.grasp_pose.pose.position.z + 0.12,
                (0.0, 0.0, 0.08), ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0),
                f'{det.class_id} {det.confidence:.2f} ({center_x:.0f}, {center_y:.0f})'
            ))
        if self.map is not None:
            markers.markers.append(self._make_marker(
                10, Marker.CUBE, self.map.header.frame_id, 0.0, 0.0, -0.01,
                (self.map.info.width * self.map.info.resolution, self.map.info.height * self.map.info.resolution, 0.02),
                ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.15)
            ))
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = RVizBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
