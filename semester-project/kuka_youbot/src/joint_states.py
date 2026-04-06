
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('arm_trajectory_commander')

pub = rospy.Publisher('/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
rospy.sleep(1)

msg = JointTrajectory()
msg.joint_names = [
    'manipulator_j1', 'manipulator_j2', 'manipulator_j3',
    'manipulator_j4', 'manipulator_j5'
]

point = JointTrajectoryPoint()
point.positions = [0.5, -0.4, 1.2, -0.8, 0.0]
point.time_from_start = rospy.Duration(3.0)

msg.points.append(point)

rospy.loginfo("Sending trajectory command...")
pub.publish(msg)





