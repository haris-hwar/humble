#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import math
import tf.transformations as tf

def create_cone_marker(frame_id, marker_id, position, orientation, height, radius, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cone"
    marker.id = marker_id
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    # Define the cone's vertices
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments

    for i in range(n_segments):
        theta = i * delta_theta

        # Apex vertex
        p1 = Point()
        p1.x = 0
        p1.y = 0
        p1.z = 0  # Apex at the bottom

        # Base vertex using the azimuthal angle
        p2 = Point()
        p2.x = radius * math.cos(theta)
        p2.y = radius * math.sin(theta)
        p2.z = height  # Base at the top

        # Next base vertex using the azimuthal angle
        p3 = Point()
        p3.x = radius * math.cos(theta + delta_theta)
        p3.y = radius * math.sin(theta + delta_theta)
        p3.z = height  # Base at the top

        marker.points.append(p3)
        marker.points.append(p2)
        marker.points.append(p1)

    # Set the pose and scale of the marker
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the color of the marker
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

if __name__ == "__main__":
    rospy.init_node("cone_publisher")
    pub = rospy.Publisher("cone_marker", Marker, queue_size=10)

    frame_id = "base_link"
    marker_id = 0
    position = Point(0, 0, 0)
    
    # Euler angles (in radians): roll, pitch, yaw
    roll = 0
    pitch = 0
    yaw = 0

    # Convert Euler angles to Quaternion
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    orientation = Quaternion(*quaternion)

    height = 1.0
    radius = 0.5
    color = (1.0, 0.0, 0.0, 1.0)  # RGBA, Red color with full opacity

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker = create_cone_marker(frame_id, marker_id, position, orientation, height, radius, color)
        pub.publish(marker)  # Publishing the marker object
        rate.sleep()
