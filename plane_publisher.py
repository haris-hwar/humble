#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

def create_plane_marker():
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "shapes"
    marker.id = 1
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 3.0
    marker.scale.y = 3.0
    marker.scale.z = 0.01
    marker.color.a = 0.5
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    return marker

def publish_plane():
    rospy.init_node('plane_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker = create_plane_marker()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_plane()
    except rospy.ROSInterruptException:
        pass
