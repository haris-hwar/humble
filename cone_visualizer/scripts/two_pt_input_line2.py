#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# def get_point_input(point_name):
#     x = float(input(f"Enter {point_name} x-coordinate: "))
#     y = float(input(f"Enter {point_name} y-coordinate: "))
#     z = float(input(f"Enter {point_name} z-coordinate: "))
#     return Point(x=x, y=y, z=z)

def publish_points_and_line(point1, point2):
    rospy.init_node("points_and_line_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a marker for the points
    points_marker = Marker()
    points_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "points_and_line"
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w = 1.0
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.scale.x = 0.1  # Point size
    points_marker.scale.y = 0.1
    points_marker.color.r = 0.0
    points_marker.color.g = 1.0  # Green color
    points_marker.color.b = 0.0
    points_marker.color.a = 1.0  # Alpha
    points_marker.points.append(Point(1,1,1))
    points_marker.points.append(Point(5,5,5))

    # moving_point = Point()
    # moving_point.x = float(point2.x - 0.5)#0.0
    # moving_point.y = float(point2.y - 0.5)#2.0
    # moving_point.z = point2.z
    
    # moving_point1 = Point()
    # moving_point1.x = 0.0
    # moving_point1.y = 2.0
    # moving_point1.z = 0.0

    # Add the points to the marker
    # points_marker.points.append(point1)
    # points_marker.points.append(point2)
    #points_marker.points.append(moving_point)
    #points_marker.points.append(moving_point1)

    # Create a marker for the line
    line_marker = Marker()
    line_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "points_and_line"
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.id = 1
    line_marker.type = Marker.LINE_STRIP
    line_marker.scale.x = 0.05  # Line width
    line_marker.color.r = 0.0
    line_marker.color.g = 0.0
    line_marker.color.b = 1.0  # Blue color
    line_marker.color.a = 1.0  # Alpha
    line_marker.points.append(Point(1,1,1))
    line_marker.points.append(Point(5,5,5))

    # Add the points to the line marker
    # line_marker.points.append(point1)
    # line_marker.points.append(point2)
    #line_marker.points.append(moving_point)

    # Publish the points and line marker
    while not rospy.is_shutdown():
        marker_pub.publish(points_marker)
        marker_pub.publish(line_marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        # # Get points from user input
        # point1 = get_point_input("Point 1")
        # point2 = get_point_input("Point 2")
        publish_points_and_line(Point,Point)
    except rospy.ROSInterruptException:
        pass
