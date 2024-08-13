#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import math

def quaternion_to_euler(quaternion):
    euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return euler

def euler_to_degrees(euler):
    return [math.degrees(angle) for angle in euler]

def corrected_pose_callback(data):
    position = data.pose.position
    orientation = data.pose.orientation
    euler = quaternion_to_euler(orientation)
    euler_degrees = euler_to_degrees(euler)
    
    print("Corrected Pose Data:")
    print("Position - x: %f, y: %f, z: %f" % (position.x, position.y, position.z))
    print("Orientation - roll: %f, pitch: %f, yaw: %f" % (euler_degrees[0], euler_degrees[1], euler_degrees[2]))

def pursuer_pose_callback(data):
    position = data.pose.position
    orientation = data.pose.orientation
    euler = quaternion_to_euler(orientation)
    euler_degrees = euler_to_degrees(euler)
    
    print("---------------------------------------------------------------------------------------------------")
    print("PURSUER Pose: (%f, %f, %f) m, (Roll=%f, Pitch=%f, Yaw=%f) deg" % (round(position.x,3), round(position.y,3), round(position.z,3), round(euler_degrees[0],3), round(euler_degrees[1],3), round(euler_degrees[2],3) ) )
    # print("Position - x: %f, y: %f, z: %f" % (position.x, position.y, position.z))
    # print("Orientation - roll: %f, pitch: %f, yaw: %f" % (euler_degrees[0], euler_degrees[1], euler_degrees[2]))

def read_poses():
    rospy.init_node('pose_reader', anonymous=True)
    rospy.Subscriber('/pursuer/corrected_pose', PoseStamped, corrected_pose_callback)
    rospy.Subscriber('/pursuer/mavros/local_position/pose', PoseStamped, pursuer_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        read_poses()
    except rospy.ROSInterruptException:
        pass
