#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3

def publish_vector():
    rospy.init_node('vector_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    rate = rospy.Rate(1)  # 10 Hz

    # Initial position of the vector start point
    start_pos = Vector3()
    start_pos.x = 0.0
    start_pos.y = 0.0
    start_pos.z = 0.0

    while not rospy.is_shutdown():
        # Create the marker
        marker = Marker()
        marker.header.frame_id = "base_link"  # Change frame as needed
        marker.header.stamp = rospy.Time.now()

        marker.ns = "vector"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Define start and end points of the arrow
        start_point = Point()
        start_point.x = start_pos.x
        start_point.y = start_pos.y
        start_point.z = start_pos.z

        end_point = Point()
        end_point.x = start_pos.x + 1.0
        end_point.y = start_pos.y + 1.0
        end_point.z = start_pos.z + 1.0
        # give the required vector here

        marker.points.append(start_point)
        marker.points.append(end_point)

        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.2  # head diameter
        marker.scale.z = 0.2  # head length

        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Publish the marker
        marker_pub.publish(marker)

        # Increment start position to translate the vector
        # start_pos.x += 0.01
        # start_pos.y += 0.01

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_vector()
    except rospy.ROSInterruptException:
        pass
