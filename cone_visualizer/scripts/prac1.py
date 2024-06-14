#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_points_and_lines(p1,p2):
    rospy.init_node("points_and_lines_in_rviz")
    pub = rospy.Publisher('points_and_lines', Marker, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    points_marker = Marker() # creating a marker object of name points_marker 
    points_marker.header.frame_id = "base_link"
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "points"
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.action = Marker.ADD
    points_marker.scale.x = 0.1
    points_marker.scale.y = 0.1
    points_marker.color.r = 0.0
    points_marker.color.g = 0.5
    points_marker.color.b = 0.5
    points_marker.color.a = 1.0
    points_marker.lifetime = rospy.Duration(0)
    points_marker.points.append(p1)
    points_marker.points.append(p2)

    line_marker = Marker()
    #fsnitasc
    line_marker.header.frame_id = "base_link"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "lines"
    line_marker.id = 1
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.05
    line_marker.scale.y = 0.05
    line_marker.color.r = 1.0
    line_marker.color.g = 0.0
    line_marker.color.b = 0.0
    line_marker.color.a = 1.0
    line_marker.points.append(p1)
    line_marker.points.append(p2)

    while not rospy.is_shutdown():
        pub.publish(points_marker) 
        # for pub.publish refer the rospy publisher name 
        #and then in the bracket pass the object name
        pub.publish(line_marker)
        rate.sleep()

if __name__ == "__main__":
    try:

        p1 = Point(x=1,y=1,z=1)
        p2 = Point(x=1,y=1,z=5)

        publish_points_and_lines(p1,p2) #call the function here
    except rospy.ROSInterruptException:
        pass
