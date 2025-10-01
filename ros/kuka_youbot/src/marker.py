

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class EndEffectorMarkerPublisher:
    def __init__(self):
        rospy.init_node('end_effector_marker_node', anonymous=True)

        # Subscriber to end effector position
        self.pose_sub = rospy.Subscriber('/end_pose', Point, self.pose_callback)

        # Publisher for the marker
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        self.fixed_frame = "base"

    def pose_callback(self, msg: Point):
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = rospy.Time.now()

        marker.ns = "end_effector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position of the marker
        marker.pose.position = msg
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Published marker at: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        EndEffectorMarkerPublisher().run()
    except rospy.ROSInterruptException:
        pass
