
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

class EndEffectorPathPublisher:
    def __init__(self):
        rospy.init_node('end_effector_path_node', anonymous=True)

        # Subscribe to end effector position
        self.pose_sub = rospy.Subscriber('/end_pose', Point, self.pose_callback)

        # Publisher for the path
        self.path_pub = rospy.Publisher('/end_effector_path', Path, queue_size=10)

        # Path message
        self.path = Path()
        self.path.header.frame_id = "base"

    def pose_callback(self, msg: Point):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base"

        pose_stamped.pose.position = msg
        pose_stamped.pose.orientation.w = 1.0 

        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_stamped)

        self.path_pub.publish(self.path)
        rospy.loginfo(f"Path updated with point: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        EndEffectorPathPublisher().run()
    except rospy.ROSInterruptException:
        pass
