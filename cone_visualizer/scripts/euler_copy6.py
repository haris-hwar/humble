#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import math

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to quaternion.
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(qx, qy, qz, qw)

def euler_from_vector(vector):
    """
    Convert a direction vector to Euler angles (in radians).
    """
    x, y, z = vector
    yaw = math.atan2(y, x)
    pitch = math.atan2(z, math.sqrt(x**2 + y**2))
    roll = 0
    return roll, pitch, yaw

def create_cone_marker(frame_id, marker_id, position, orientation, height, vertex_angle, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cone"
    marker.id = marker_id
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    # Calculate the base radius from the vertex angle
    #radius = height * math.tan(vertex_angle / 2.0)

    # Define the cone's vertices
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments
    slant_height = math.sqrt(height**2 + radius**2)

    # Calculate height of the cap
    cap_height = slant_height - math.sqrt(slant_height**2 - radius**2)

    for i in range(n_segments):
        theta = i * delta_theta
        
        # Center of the base of the cone 
        p1 = Point()
        p1.x = 0
        p1.y = 0
        p1.z = 0

        # Base vertex using the azimuthal angle
        p2 = Point()
        p2.x = height - cap_height
        p2.y = radius * math.cos(theta)
        p2.z = radius * math.sin(theta)

        # Next base vertex using the azimuthal angle
        p3 = Point()
        p3.x = height - cap_height
        p3.y = radius * math.cos(theta + delta_theta)
        p3.z = radius * math.sin(theta + delta_theta)

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

def create_spherical_cap_marker(frame_id, marker_id, position, orientation, height, radius, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "spherical_cap"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    # Calculate slant height of cone
    slant_height = math.sqrt(height**2 + radius**2)

    # Calculate height of the cap
    cap_height = slant_height - math.sqrt(slant_height**2 - radius**2)

    # Calculate the radius of the base of the spherical cap
    base_radius = math.sqrt(slant_height**2 - (slant_height - cap_height)**2)

    # Define the spherical cap's vertices
    n_segments = 180
    delta_theta = 2 * math.pi / n_segments
    delta_phi = math.pi / 360  # Increase the density of points along the height

    for i in range(n_segments):
        theta1 = i * delta_theta
        theta2 = (i + 1) % n_segments * delta_theta
        for j in range(int(math.pi / delta_phi)):
            phi1 = j * delta_phi
            phi2 = (j + 1) * delta_phi

            if phi1 > math.asin(base_radius / slant_height):
                break

            p1 = Point()
            p1.x = slant_height * math.cos(phi1) - cap_height
            p1.y = slant_height * math.sin(phi1) * math.cos(theta1)
            p1.z = slant_height * math.sin(phi1) * math.sin(theta1)
            
            p2 = Point()
            p2.x = slant_height * math.cos(phi1) - cap_height
            p2.y = slant_height * math.sin(phi1) * math.cos(theta2)
            p2.z = slant_height * math.sin(phi1) * math.sin(theta2)

            p3 = Point()
            p3.x = slant_height * math.cos(phi2) - cap_height
            p3.y = slant_height * math.sin(phi2) * math.cos(theta1)
            p3.z = slant_height * math.sin(phi2) * math.sin(theta1)

            p4 = Point()
            p4.x = slant_height * math.cos(phi2) -  cap_height
            p4.y = slant_height * math.sin(phi2) * math.sin(theta2)
            p4.z = slant_height * math.sin(phi2) * math.cos(theta2)

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

    # Define the vector to which the spherical sector should point
    target_vector = (1, 0, 0)

    target_vector = (target_vector[0], target_vector[1], -target_vector[2])

    # Computing the Euler angles from the target vector
    roll, pitch, yaw = euler_from_vector(target_vector)
    orientation = quaternion_from_euler(roll, pitch, yaw)

    # Length of the vector
    # length = math.sqrt(((target_vector[0])**2) + ((target_vector[1])**2) + ((target_vector[2])**2))

    #r_ix = radius of the spherical sector as per gse
    cone_height = 1.0 
    radius = 0.5
    r_ix = cone_height
    l_ix = radius
    # l_ix = distance between p_ix i.e., the point on obstacle and q_ix i.e.,
    # maximum distance of point p_ix from the points in q_ix can be given as l_ix
    # vertex_angle = math.radians(60)  # Define the vertex angle in radians
    # Calculate the vertex angle
    vertex_angle = 2 * math.atan2(l_ix/r_ix)

    color = (1.0, 0.0, 0.0, 1.0)  # RGBA

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        cone_marker = create_cone_marker(frame_id, 0, position, orientation, cone_height, vertex_angle, color)
         
        cap_marker = create_spherical_cap_marker(frame_id, 1, position, orientation, cone_height, cone_height * math.tan(vertex_angle / 2.0), color)

        pub.publish(cone_marker)
        pub.publish(cap_marker)

        rate.sleep()
