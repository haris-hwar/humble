#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

def create_marker(marker_id, marker_type, frame_id, position, orientation, scale, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "markers"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale = scale
    marker.color = color
    marker.lifetime = rospy.Duration()
    return marker

def main():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        # Define position and orientation
        position = Point(1.0, 2.0, 0.0)
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        # Define marker scale
        scale = Point(0.2, 0.2, 0.2)

        # Define marker color
        color = Marker().color
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
        color.a = 1.0

        # Create and publish a point marker
        point_marker = create_marker(0, Marker.SPHERE, "map", position, orientation, scale, color)
        marker_pub.publish(point_marker)

        # Create and publish an arrow marker
        arrow_marker = create_marker(1, Marker.ARROW, "map", position, orientation, Point(1.0, 0.1, 0.1), color)
        marker_pub.publish(arrow_marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


