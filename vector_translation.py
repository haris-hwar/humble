#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point

def translate_vector(vector1, vector2):
    translated_vector = Vector3()
    translated_vector.x = vector2.x - vector1.x 
    translated_vector.y = vector2.y - vector1.y 
    translated_vector.z = vector2.z - vector1.z 
    return translated_vector

def create_marker(vector, marker_id, color, frame_id="world"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "vectors"
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    marker.points = []
    start_point = Point()
    start_point.x = 0
    start_point.y = 0
    start_point.z = 0

    end_point = Point()
    end_point.x = vector.x
    end_point.y = vector.y
    end_point.z = vector.z

    marker.points.append(start_point)
    marker.points.append(end_point)

    return marker

def visualize_vectors(vector1, vector2, translated_vector):
    rospy.init_node('vector_visualizer')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(1)  # Wait a bit for the publisher to set up

    marker1 = create_marker(vector1, 1, [1.0, 0.0, 0.0])  # Red for vector1
    marker2 = create_marker(vector2, 2, [0.0, 1.0, 0.0])  # Green for vector2
    # marker3 = create_marker(translated_vector, 3, [0.0, 0.0, 1.0])  # Blue for translated vector

    while not rospy.is_shutdown():
        marker_pub.publish(marker1)
        marker_pub.publish(marker2)
        # marker_pub.publish(marker3)
        rospy.sleep(0.1)  # Publish at 10 Hz

if __name__ == '__main__':
    vector1 = Vector3(1, 1, 1)  # At origin
    vector2 = Vector3(-3, -3, 3)  # Example coordinates
    translated_vector = translate_vector(vector1, vector2)
    visualize_vectors(vector1, vector2, translated_vector)
