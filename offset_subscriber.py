#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def corrected_pose_callback(data):
    rospy.loginfo("Corrected Pose Data:")
    rospy.loginfo("Position - x: %f, y: %f, z: %f", data.pose.position.x, data.pose.position.y, data.pose.position.z)
    rospy.loginfo("Orientation - x: %f, y: %f, z: %f, w: %f", 
                  data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

def read_corrected_pose():
    rospy.init_node('corrected_pose_reader', anonymous=True)
    rospy.Subscriber('/target/corrected_pose', PoseStamped, corrected_pose_callback)
    rospy.spin()

def pursuer_pose_callback(data):
    rospy.loginfo("pursuer pose data:")
    rospy.loginfo("Position - x: %f, y: %f, z: %f", data.pose.position.x, data.pose.position.y, data.pose.position.z)
    rospy.loginfo("Orientation - x: %f, y: %f, z: %f, w: %f", 
                  data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

def read_pursuer_pose():
    rospy.init_node('pursuer_pose_reader', anonymous=True)
    rospy.Subscriber('/pursuer/mavros/local_position/pose', PoseStamped, pursuer_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        read_corrected_pose()
        read_pursuer_pose()
    except rospy.ROSInterruptException:
        pass
