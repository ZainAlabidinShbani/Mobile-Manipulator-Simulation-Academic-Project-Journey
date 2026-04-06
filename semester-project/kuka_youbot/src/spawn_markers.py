#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import random

def spawn_five_cubes():
    rospy.init_node('spawn_five_cubes_node')
    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    rospy.sleep(1)

    marker_array = MarkerArray()
    cube_size = 0.05
    colors = [
        (1.0, 0.0, 0.0),   
        (0.0, 1.0, 0.0),   
        (0.0, 0.0, 1.0),  
        (1.0, 1.0, 0.0),   
        (1.0, 0.0, 1.0),   
    ]

    positions = [
    (1.3, 1.8),
    (-2.3, 2.8),
    (-2.3, -2.3),
    (0.3, -2.0),
    (3.2, 0.9)
    ]


    for i in range(5):
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cubes"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = positions[i][0]
        marker.pose.position.y = positions[i][1]
        marker.pose.position.z = cube_size / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = cube_size
        marker.scale.y = cube_size
        marker.scale.z = cube_size
        marker.color.r = colors[i][0]
        marker.color.g = colors[i][1]
        marker.color.b = colors[i][2]
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)  # دائم
        marker_array.markers.append(marker)
        

        # Add drop-off target base (circular platform)
    target_marker = Marker()
    target_marker.header.frame_id = "base"
    target_marker.header.stamp = rospy.Time.now()
    target_marker.ns = "target_base"
    target_marker.id = 100
    target_marker.type = Marker.CYLINDER
    target_marker.action = Marker.ADD
    target_marker.pose.position.x = 0.0
    target_marker.pose.position.y = 3.0
    target_marker.pose.position.z = 0.05  # small height to appear on ground
    target_marker.pose.orientation.x = 0.0
    target_marker.pose.orientation.y = 0.0
    target_marker.pose.orientation.z = 0.0
    target_marker.pose.orientation.w = 1.0
    target_marker.scale.x = 0.3
    target_marker.scale.y = 0.3
    target_marker.scale.z = 0.02  # flat disc
    target_marker.color.r = 0.2
    target_marker.color.g = 0.2
    target_marker.color.b = 1.0
    target_marker.color.a = 1.0
    target_marker.lifetime = rospy.Duration(0)

    marker_array.markers.append(target_marker)


    pub.publish(marker_array)
    rospy.loginfo("Spawned 5 cubes.")

if __name__ == "__main__":
    spawn_five_cubes()
