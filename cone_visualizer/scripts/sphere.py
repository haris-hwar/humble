#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def draw_sphere():
    rospy.init_node('sphere_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('sphere_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # Rate at which to publish marker (1 Hz)

    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "map"  # Frame ID in which the marker will be displayed
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 1.0  # Diameter of the sphere
    marker.scale.y = 0.5
    marker.scale.z = 1.0
    marker.color.a = 1.0  # Full opacity
    marker.color.r = 0.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 1.0  # Blue
    marker.pose.position.x = 0.0  # Position of the center of the sphere
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        draw_sphere()
    except rospy.ROSInterruptException:
        pass
