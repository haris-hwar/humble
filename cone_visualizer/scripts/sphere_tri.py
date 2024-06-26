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
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments

    for i in range(n_segments):
        theta = i * delta_theta

        # Apex vertex
        p1 = Point()
        p1.x = 0
        p1.y = 0
        p1.z = height

        # Base vertex using the azimuthal angle
        p2 = Point()
        p2.x = radius * math.cos(theta)
        p2.y = radius * math.sin(theta)
        p2.z = 0

        # Next base vertex using the azimuthal angle
        p3 = Point()
        p3.x = radius * math.cos(theta + delta_theta)
        p3.y = radius * math.sin(theta + delta_theta)
        p3.z = 0

        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)

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

def create_spherical_cap_marker(frame_id, marker_id, position, orientation, cone_height, cone_radius, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "spherical_cap"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    # Calculate slant height of cone
    slant_height = math.sqrt(cone_height**2 + cone_radius**2)

    # Calculate height of the cap
    cap_height = slant_height - math.sqrt(slant_height**2 - cone_radius**2)

    # Calculate the radius of the base of the spherical cap
    base_radius = math.sqrt(slant_height**2 - (slant_height - cap_height)**2)

    # Define the spherical cap's vertices
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments
    delta_phi = math.pi / 18  # Increase the density of points along the height

    for i in range(n_segments):
        theta1 = i * delta_theta
        theta2 = (i + 1) % n_segments * delta_theta
        for j in range(int(math.pi / delta_phi)):
            phi1 = j * delta_phi
            phi2 = (j + 1) * delta_phi

            if phi1 > math.asin(base_radius / slant_height):
                break

            # Points on the spherical cap surface
            p1 = Point()
            p1.x = slant_height * math.sin(phi1) * math.cos(theta1)
            p1.y = slant_height * math.sin(phi1) * math.sin(theta1)
            p1.z = slant_height * math.cos(phi1)

            p2 = Point()
            p2.x = slant_height * math.sin(phi1) * math.cos(theta2)
            p2.y = slant_height * math.sin(phi1) * math.sin(theta2)
            p2.z = slant_height * math.cos(phi1)

            p3 = Point()
            p3.x = slant_height * math.sin(phi2) * math.cos(theta1)
            p3.y = slant_height * math.sin(phi2) * math.sin(theta1)
            p3.z = slant_height * math.cos(phi2)

            p4 = Point()
            p4.x = slant_height * math.sin(phi2) * math.cos(theta2)
            p4.y = slant_height * math.sin(phi2) * math.sin(theta2)
            p4.z = slant_height * math.cos(phi2)

            marker.points.append(p1)
            marker.points.append(p2)
            marker.points.append(p3)
            marker.points.append(p1)
            marker.points.append(p3)
            marker.points.append(p4)

    # Set the pose and scale of the marker
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale.x = 0.01  # Line width

    # Set the color of the marker
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

if __name__ == "__main__":
    rospy.init_node("spherical_sector_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    frame_id = "base_link"
    position = Point(0, 0, 0)
    orientation = Quaternion(0, 0, 0, 1)
    cone_height = 1.0
    cone_radius = 0.5
    color = (1.0, 0.0, 0.0, 1.0)  # RGBA

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Create cone marker
        cone_marker = create_cone_marker(frame_id, 0, position, orientation, cone_height, cone_radius, color)

        # Create spherical cap marker
        cap_marker = create_spherical_cap_marker(frame_id, 1, position, orientation, cone_height, cone_radius, color)

        # Publish markers
        pub.publish(cone_marker)
        pub.publish(cap_marker)

        rate.sleep()
