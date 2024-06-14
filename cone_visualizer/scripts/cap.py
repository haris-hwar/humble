#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

import math

def rotate_point(point, orientation):
    # Function to rotate a point by a quaternion orientation
    q = orientation
    rot_matrix = [
        [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
        [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
        [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
    ]

    # Apply rotation
    x_new = rot_matrix[0][0] * point.x + rot_matrix[0][1] * point.y + rot_matrix[0][2] * point.z
    y_new = rot_matrix[1][0] * point.x + rot_matrix[1][1] * point.y + rot_matrix[1][2] * point.z
    z_new = rot_matrix[2][0] * point.x + rot_matrix[2][1] * point.y + rot_matrix[2][2] * point.z

    return Point(x_new, y_new, z_new)

def visualize_spherical_cap():
    rospy.init_node('spherical_cap_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # Publish rate in Hz

    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "base_link"  # Specify the frame ID
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Parameters for the spherical cap
    radius = 1.0
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments
    delta_phi = math.pi / 18

    # Spherical cap vertices
    for i in range(n_segments):
        theta1 = i * delta_theta
        theta2 = (i + 1) % n_segments * delta_theta
        for j in range(int(math.pi / delta_phi)):
            phi1 = j * delta_phi
            phi2 = (j + 1) * delta_phi

            z1 = radius * math.cos(phi1)
            z2 = radius * math.cos(phi2)
            if z1 < 0 or z2 < 0:
                continue

            p1 = Point()
            p1.x = radius * math.sin(phi1) * math.cos(theta1)
            p1.y = radius * math.sin(phi1) * math.sin(theta1)
            p1.z = radius * math.cos(phi1)

            p2 = Point()
            p2.x = radius * math.sin(phi1) * math.cos(theta2)
            p2.y = radius * math.sin(phi1) * math.sin(theta2)
            p2.z = radius * math.cos(phi1)

            p3 = Point()
            p3.x = radius * math.sin(phi2) * math.cos(theta1)
            p3.y = radius * math.sin(phi2) * math.sin(theta1)
            p3.z = radius * math.cos(phi2)

            p4 = Point()
            p4.x = radius * math.sin(phi2) * math.cos(theta2)
            p4.y = radius * math.sin(phi2) * math.sin(theta2)
            p4.z = radius * math.cos(phi2)

            # Rotate points if needed
            p1 = rotate_point(p1, Quaternion(0, 0, 0, 1))  # Provide orientation if rotation is needed
            p2 = rotate_point(p2, Quaternion(0, 0, 0, 1))
            p3 = rotate_point(p3, Quaternion(0, 0, 0, 1))
            p4 = rotate_point(p4, Quaternion(0, 0, 0, 1))

            marker.points.append(p1)
            marker.points.append(p2)
            marker.points.append(p3)
            marker.points.append(p1)
            marker.points.append(p3)
            marker.points.append(p4)

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        visualize_spherical_cap()
    except rospy.ROSInterruptException:
        pass

