#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

def create_spherical_sector_marker(frame_id, r, h, a, phi_a, theta_a):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.01  # Line width
    marker.color.a = 1.0  # Alpha
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    alpha = math.atan(a / (r - h))
    
    for rho in [r * i / 10.0 for i in range(11)]:
        for phi in [phi_a - alpha / 2, phi_a + alpha / 2]:
            for theta in [theta_a - alpha / 2, theta_a + alpha / 2]:
                x = rho * math.cos(theta) * math.sin(phi)
                y = rho * math.sin(theta) * math.sin(phi)
                z = rho * math.cos(phi)
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                marker.points.append(p)

    return marker

if __name__ == '__main__':
    rospy.init_node('spherical_sector_marker')

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    r = 1.0  #  radius
    h = 0.5  #  height
    a = 0.5  #  base radius
    phi_a = 0.5  # polar angle
    theta_a = 0.5  # azimuthal angle

    marker = create_spherical_sector_marker('base_link', r, h, a, phi_a, theta_a)
    
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rospy.sleep(0.1)
