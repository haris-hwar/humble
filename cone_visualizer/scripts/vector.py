#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_vector():
    rospy.init_node('vector_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "vector"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Define start and end points of the arrow
        start_point = Point()
        start_point.x = 1
        start_point.y = 1
        start_point.z = 1

        end_point = Point()
        end_point.x = 0
        end_point.y = 0
        end_point.z = 1

        marker.points.append(start_point)
        marker.points.append(end_point)

        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.2  # head diameter
        marker.scale.z = 0.2  # head length

        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_vector()
    except rospy.ROSInterruptException:
        pass
