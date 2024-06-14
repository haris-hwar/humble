#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import math

def create_cone_marker(frame_id, marker_id, position, orientation, height, radius, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cone"
    marker.id = marker_id
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    # Define the cone's vertices
    n_segments = 18  # Number of triangle segments to approximate the cone
    theta = 0
    delta_theta = 2 * 3.14159265359 / n_segments

    for i in range(n_segments):
        # Base vertex
        p1 = Point()
        p1.x = radius * math.cos(theta)
        p1.y = radius * math.sin(theta)
        p1.z = 0

        # Next base vertex
        p2 = Point()
        p2.x = radius * math.cos(theta + delta_theta)
        p2.y = radius * math.sin(theta + delta_theta)
        p2.z = 0

        # Apex vertex
        p3 = Point()
        p3.x = 0
        p3.y = 0
        p3.z = height

        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)

        theta += delta_theta

    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

if __name__ == "__main__":
    rospy.init_node("cone_marker_publisher")

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    frame_id = "base_link"
    marker_id = 0
    position = Point(0, 0, 1)
    orientation = Quaternion(1, 0, 0, 0)
    height = 1.0
    radius = 0.5
    color = (0.0, 1.0, 0.0, 1.0)  # RGBA

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker = create_cone_marker(frame_id, marker_id, position, orientation, height, radius, color)
        marker_pub.publish(marker)
        rate.sleep()
