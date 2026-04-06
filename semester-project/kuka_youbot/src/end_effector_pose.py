import rospy
import tf
from geometry_msgs.msg import PointStamped, Point

class EndEffectorTF:
    def __init__(self):
        rospy.init_node('end_effector_tf_reader', anonymous=True)
        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher('/end_pose', Point, queue_size=10)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/base', '/gripper_base', rospy.Time(0))

                point = Point(x=trans[0], y=trans[1], z=trans[2])
                self.pose_pub.publish(point)
                rospy.loginfo(f"[TF Gripper] x: {point.x:.3f}, y: {point.y:.3f}, z: {point.z:.3f}")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF not available.")
                continue
            rate.sleep()

if __name__ == '__main__':
    try:
        EndEffectorTF().run()
    except rospy.ROSInterruptException:
        pass
