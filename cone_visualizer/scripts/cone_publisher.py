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

def create_cone_marker(frame_id, marker_id, position, orientation, length, radius):
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
        p2 = Point(length, radius * math.cos(theta), radius * math.sin(theta))
        p3 = Point(length, radius * math.cos(theta + delta_theta), radius * math.sin(theta + delta_theta))
        marker.points.extend([p3, p2, p1])

    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0, 1.0, 0, 0.5)
    return marker

# def create_plane_marker(frame_id, marker_id):
#     marker = Marker()
#     marker.header.frame_id = frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "plane"
#     marker.id = marker_id
#     marker.type = Marker.CUBE
#     marker.action = Marker.ADD
#     marker.pose.position = Point(0, 0, 0)
#     marker.pose.orientation = Quaternion(0, 0, 0, 1)
#     marker.scale.x = marker.scale.y = 10.0
#     marker.scale.z = 0.1
#     marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.


def main():
    rospy.init_node('cone_publisher')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    frame_id = "base_link"
    marker_id = 0
    position = Point(0, 0, 0)
    orientation = quaternion_from_euler(0, 0, 0)
    length = 2
    radius = 0.5
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        cone_marker = create_cone_marker(frame_id, 0, position, orientation, length, radius)
        pub.publish(cone_marker)
        rate.sleep()

if __name__ == '__main__':
    main()
