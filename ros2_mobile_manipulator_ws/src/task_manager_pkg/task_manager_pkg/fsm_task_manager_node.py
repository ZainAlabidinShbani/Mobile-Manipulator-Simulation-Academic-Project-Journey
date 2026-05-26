import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mobile_manipulator_msgs.action import ExecutePickPlace
from mobile_manipulator_msgs.msg import DetectionArray, RobotState
from mobile_manipulator_msgs.srv import ChangeMode
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FsmTaskManagerNode(Node):
    def __init__(self):
        super().__init__('fsm_task_manager_node')
        self.mode = 'idle'
        self.task_state = 'waiting'
        self.latest_detections = None
        self.latest_pose = None
        self.latest_joints = []
        self.latest_goal_class = ''
        self.robot_state_pub = self.create_publisher(RobotState, '/robot_state', 10)
        self.navigate_pub = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_target_pose', 10)
        self.dropoff_pub = self.create_publisher(PoseStamped, '/dropoff_target_pose', 10)
        self.backoff_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(DetectionArray, '/detections', self._detections_cb, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self._pose_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joints_cb, 10)
        self.mode_srv = self.create_service(ChangeMode, 'change_mode', self._change_mode_cb)
        self.action_server = ActionServer(
            self,
            ExecutePickPlace,
            'execute_pick_place',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.timer = self.create_timer(0.2, self._publish_state)

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _detections_cb(self, msg):
        self.latest_detections = msg

    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _joints_cb(self, msg):
        self.latest_joints = list(msg.position)

    def _change_mode_cb(self, request, response):
        self._set_mode(request.mode, 'mode update request')
        response.success = True
        response.message = f'Mode switched to {request.mode}.'
        return response

    def _set_mode(self, mode, task_state):
        self.mode = mode
        self.task_state = task_state

    def _publish_state(self):
        state = RobotState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.mode = self.mode
        state.task_state = self.task_state
        state.pose = self.latest_pose if self.latest_pose is not None else PoseStamped()
        state.joint_positions = list(self.latest_joints)
        state.perception_ok = self.latest_detections is not None and bool(self.latest_detections.detections)
        state.navigation_ok = self.latest_pose is not None
        state.manipulation_ok = len(self.latest_joints) >= 4
        self.robot_state_pub.publish(state)

    def _wait_for_condition(self, predicate, timeout_sec, phase, feedback, goal_handle):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            if predicate():
                return True
            feedback.phase = phase
            feedback.detail = 'waiting'
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        return predicate()

    def _nearest_detection(self, target_class):
        if self.latest_detections is None or not self.latest_detections.detections:
            return None
        matches = [det for det in self.latest_detections.detections if not target_class or det.class_id == target_class]
        return matches[0] if matches else None

    def _goal_to_pregrasp(self, pose):
        goal = PoseStamped()
        goal.header = pose.header
        goal.pose = pose.pose
        goal.pose.position.x = pose.pose.position.x - 0.45
        goal.pose.position.z = max(0.15, pose.pose.position.z)
        return goal

    def _goal_to_dropoff(self, pose):
        goal = PoseStamped()
        goal.header = pose.header
        goal.pose = pose.pose
        return goal

    def _publish_navigation_goal(self, pose):
        self.navigate_pub.publish(pose)

    def _publish_arm_target(self, pose, is_dropoff=False):
        if is_dropoff:
            self.dropoff_pub.publish(pose)
        else:
            self.grasp_pub.publish(pose)

    def _backoff(self):
        twist = Twist()
        twist.linear.x = -0.08
        self.backoff_pub.publish(twist)
        time.sleep(0.5)
        self.backoff_pub.publish(Twist())

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = ExecutePickPlace.Feedback()
        result = ExecutePickPlace.Result()
        self.latest_goal_class = goal.target_class

        self._set_mode('perception', 'searching for target')
        if not self._wait_for_condition(lambda: self._nearest_detection(goal.target_class) is not None, 6.0, 'perception', feedback, goal_handle):
            self.task_state = 'perception_failed'
            result.success = False
            result.message = 'Perception failed after retries.'
            goal_handle.abort()
            return result

        det = self._nearest_detection(goal.target_class)
        target_pose = goal.target_pose if goal.target_pose.header.frame_id else det.grasp_pose
        dropoff_pose = goal.dropoff_pose if goal.dropoff_pose.header.frame_id else det.grasp_pose
        pregrasp = self._goal_to_pregrasp(target_pose)

        self._set_mode('navigation', 'moving to pick location')
        self._publish_navigation_goal(pregrasp)
        if not self._wait_for_condition(lambda: self._close_to(pregrasp, 0.22), 20.0, 'navigation', feedback, goal_handle):
            self._set_mode('recovery', 'navigation recovery')
            self._backoff()
            self._publish_navigation_goal(pregrasp)
            if not self._wait_for_condition(lambda: self._close_to(pregrasp, 0.22), 20.0, 'navigation_recovery', feedback, goal_handle):
                result.success = False
                result.message = 'Navigation failed after recovery attempt.'
                goal_handle.abort()
                return result

        self._set_mode('manipulation', 'executing pick')
        self._publish_arm_target(target_pose, is_dropoff=False)
        time.sleep(3.5)

        self._set_mode('transport', 'moving to dropoff location')
        self._publish_navigation_goal(self._goal_to_dropoff(dropoff_pose))
        if not self._wait_for_condition(lambda: self._close_to(dropoff_pose, 0.30), 20.0, 'transport', feedback, goal_handle):
            self._set_mode('recovery', 'transport recovery')
            self._backoff()
            self._publish_navigation_goal(self._goal_to_dropoff(dropoff_pose))
            if not self._wait_for_condition(lambda: self._close_to(dropoff_pose, 0.30), 20.0, 'transport_recovery', feedback, goal_handle):
                result.success = False
                result.message = 'Transport failed after recovery attempt.'
                goal_handle.abort()
                return result

        self._set_mode('manipulation', 'executing place')
        self._publish_arm_target(dropoff_pose, is_dropoff=True)
        time.sleep(2.5)
        self._set_mode('complete', 'task complete')
        result.success = True
        result.message = 'Pick-and-place task completed successfully.'
        goal_handle.succeed()
        return result

    def _close_to(self, pose, tolerance):
        if self.latest_pose is None:
            return False
        dx = pose.pose.position.x - self.latest_pose.pose.position.x
        dy = pose.pose.position.y - self.latest_pose.pose.position.y
        return math.hypot(dx, dy) <= tolerance


def main():
    rclpy.init()
    node = FsmTaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
