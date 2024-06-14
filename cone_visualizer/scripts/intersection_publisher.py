#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def compute_intersection():
    points = []
    # Sample points along the intersection curve
    for theta in np.linspace(0, 2*np.pi, 100):
        x = np.cos(theta)
        y = np.sin(theta)
        z = x + y  # Plane z = x + y intersecting with cone z^2 = x^2 + y^2
        points.append(Point(x, y, z))
    return points

def create_intersection_marker(points):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "intersection"
    marker.id = 2
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.points = points
    return marker

def publish_intersection():
    rospy.init_node('intersection_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        points = compute_intersection()
        marker = create_intersection_marker(points)
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_intersection()
    except rospy.ROSInterruptException:
        pass
