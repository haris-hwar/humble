#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import math

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(qx, qy, qz, qw)

def euler_from_vector(vector):
    x, y, z = vector.x, vector.y, -vector.z
    yaw = math.atan2(y, x)
    pitch = math.atan2(z, math.sqrt(x**2 + y**2))
    roll = 0
    return roll, pitch, yaw

def calculate_geometry(X, Pi_x, li_x):
    height = math.sqrt((Pi_x.x - X.x)**2 + (Pi_x.y - X.y)**2 + (Pi_x.z - X.z)**2)
    slant_height = math.sqrt(height**2 + li_x**2)
    cap_height = slant_height - math.sqrt(slant_height**2 - li_x**2)
    height = height - cap_height
    slant_height = math.sqrt(height**2 + li_x**2)  # overwriting by the reduced height of the cone with the spherical cap
    cap_height = slant_height - math.sqrt(slant_height**2 - li_x**2)
    cap_radius = math.sqrt(slant_height**2 - (slant_height - cap_height)**2)
    return slant_height, cap_height, cap_radius, height, li_x

def calculate_orientation_and_height(X, Pi_x, li_x):
    roll, pitch, yaw = euler_from_vector(Pi_x)
    orientation = quaternion_from_euler(roll, pitch, yaw)
    slant_height, cap_height, cap_radius, height, li_x = calculate_geometry(X, Pi_x, li_x)
    ri_x = height 
    vertex_angle = 2 * math.atan2(li_x, ri_x)  # Compute vertex angle
    return orientation, ri_x, vertex_angle

def create_cone_marker(frame_id, marker_id, X, orientation, ri_x, li_x):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cone"
    marker.id = marker_id
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments

    for i in range(n_segments):
        theta = i * delta_theta
        p1 = Point(0, 0, 0)
        p2 = Point(ri_x, li_x * math.cos(theta), li_x * math.sin(theta))
        p3 = Point(ri_x, li_x * math.cos(theta + delta_theta), li_x * math.sin(theta + delta_theta))
        marker.points.extend([p3, p2, p1])

    marker.pose.position = X
    marker.pose.orientation = orientation
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0, 1.0, 0, 0.5)
    return marker

def create_spherical_cap_marker(frame_id, marker_id, X, orientation, slant_height, cap_height, cap_radius, height):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "spherical_cap"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    n_segments = 90
    delta_theta = 2 * math.pi / n_segments
    delta_phi = math.pi / 360

    for i in range(n_segments):
        theta1 = i * delta_theta
        theta2 = (i + 1) % n_segments * delta_theta
        for j in range(int(math.pi / delta_phi)):
            phi1 = j * delta_phi
            phi2 = (j + 1) * delta_phi

            if phi1 > math.asin(cap_radius / slant_height):
                break

            p1 = Point(slant_height * math.cos(phi1), slant_height * math.sin(phi1) * math.cos(theta1), slant_height * math.sin(phi1) * math.sin(theta1))
            p2 = Point(slant_height * math.cos(phi1), slant_height * math.sin(phi1) * math.cos(theta2), slant_height * math.sin(phi1) * math.sin(theta2))
            p3 = Point(slant_height * math.cos(phi2), slant_height * math.sin(phi2) * math.cos(theta1), slant_height * math.sin(phi2) * math.sin(theta1))
            p4 = Point(slant_height * math.cos(phi2), slant_height * math.sin(phi2) * math.sin(theta2), slant_height * math.sin(phi2) * math.cos(theta2))

            marker.points.extend([p1, p2, p3, p1, p3, p4])

    marker.pose.position = X
    marker.pose.orientation = orientation
    marker.scale.x = 0.01
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0, 0, 0.25)
    return marker

def main():
    rospy.init_node("spherical_sector_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    frame_id = "base_link"
    X = Point(0, 0, 2)
    Pi_x = Point(0, 0, 0)
    li_x = 2.0

    orientation, ri_x, vertex_angle = calculate_orientation_and_height(X, Pi_x, li_x)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        cone_marker = create_cone_marker(frame_id, 0, X, orientation, ri_x, li_x)
        slant_height, cap_height, cap_radius, _, _ = calculate_geometry(X, Pi_x, li_x)
        cap_marker = create_spherical_cap_marker(frame_id, 1, X, orientation, slant_height, cap_height, cap_radius, ri_x)
        pub.publish(cone_marker)
        pub.publish(cap_marker)
        rate.sleep()

if __name__ == "__main__":
    main()
