#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from PrintColours import *
from math import radians, cos, sin, pi, degrees
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from py_gnc_functions import gnc_api

class CorrectedPosePublisher:
    def __init__(self, namespace='pursuer',Offset=Point(0,0,0)):
        # rospy.loginfo(CBLUE + f"I am in init"+ CEND)
        self.initialOffset = Offset
        self.CorrectedPose_Pub = rospy.Publisher(f'/{namespace}/corrected_pose', PoseStamped, queue_size=10)
        self.localPos_sub = rospy.Subscriber(f'/{namespace}/mavros/global_position/local', Odometry, callback=self.localPos_Callback)
    
    def localPos_Callback(self, msg):

        # rospy.loginfo(CBLUE + f"I am in localPos_Callback"+ CEND)
        CorrectedPose_msg = self.ComputeCorrectedPose(msg)
        # print("TRANSFORMED from localPos_Callback:====>   (",CorrectLocalPose.x,", ",CorrectLocalPose.y,", ",CorrectLocalPose.z,")")
        # CorrectedPose_msg = PoseStamped()
        CorrectedPose_msg.header.stamp = rospy.Time.now()
        CorrectedPose_msg.header.frame_id = "map"
        # CorrectedPose_msg.pose.orientation = self.gnc.current_pose_g.pose.pose.orientation
        self.CorrectedPose_Pub.publish(CorrectedPose_msg)

    def ComputeCorrectedPose(self, msg):
        # rospy.loginfo(CBLUE + f"I am in ComputeCorrectedPose"+ CEND)
        CurrentLocalPos = msg.pose.pose.position
        x = CurrentLocalPos.x #+ Offset_x
        y = CurrentLocalPos.y #+ Offset_y
        z = CurrentLocalPos.z #+ Offset_z
        # print("OFFSET :====>   (",Offset_x,", ",Offset_y,", ",Offset_z,")" + f"{namespace}")
        
        CurrentLocalOrientation = msg.pose.pose.orientation
        CurrentLocalOrientation_quaternion = (
            CurrentLocalOrientation.x,
            CurrentLocalOrientation.y,
            CurrentLocalOrientation.z,
            CurrentLocalOrientation.w
        )
        (CurrentLocalRoll, CurrentLocalPitch, CurrentLocalYaw) = euler_from_quaternion(CurrentLocalOrientation_quaternion)
        # print("CurrentLocalYaw:",degrees(CurrentLocalYaw))
        transformed_yaw = CurrentLocalYaw - (pi/2)
        # print("CORRECTED LocalYaw:",degrees(transformed_yaw))
        TransformedLocalOrientation_quaternion = quaternion_from_euler(CurrentLocalRoll, CurrentLocalPitch, transformed_yaw)

        x_transformed = x * cos(pi/2) + y * sin(pi/2)
        y_transformed = -1*x * sin(pi/2) + y * cos(pi/2)
        z_transformed = z
        # print("JUST ROTATED:====>   (",x_transformed,", ",y_transformed,", ",z_transformed,")" + f"{namespace}")

        x_transformed += Offset_x
        y_transformed += Offset_y
        z_transformed += Offset_z

        # print("ROTATED + OFFSETED:====>   (",x_transformed,", ",y_transformed,", ",z_transformed,")" + f"{namespace}")
        # print("x_transformed:",x_transformed)
        # print("y_transformed:",y_transformed)
        # print("z_transformed:",z_transformed)
        # print("CurrentLocalYaw:",degrees(CurrentLocalYaw))
        # print("transformed_yaw:",degrees(transformed_yaw))
     
        CurrentLocalPos_Transformed = Point(x_transformed, y_transformed, z_transformed)
        CurrentLocalOrientation_Transformed = TransformedLocalOrientation_quaternion

        CurrentLocalPose_Transformed = PoseStamped()
        CurrentLocalPose_Transformed.pose.position = CurrentLocalPos_Transformed
        CurrentLocalPose_Transformed.pose.orientation = Quaternion(
            x=CurrentLocalOrientation_Transformed[0],
            y=CurrentLocalOrientation_Transformed[1],
            z=CurrentLocalOrientation_Transformed[2],
            w=CurrentLocalOrientation_Transformed[3]
        )

        return CurrentLocalPose_Transformed

    # def get_pose(self, x, y, z, psi):
    #     """Command the drone to fly to a waypoint in the local reference frame."""
    #     self.gnc.set_heading(psi)

    #     rospy.loginfo(CRED2 + f"Untransformed x_local x:{x} y:{y} z:{z}" + CEND)
    #     theta = radians(self.gnc.local_offset_g)
    #     Xlocal = x * cos(theta) - y * sin(theta)
    #     Ylocal = x * sin(theta) + y * cos(theta)
    #     Zlocal = z
    #     rospy.loginfo(CRED2 + f"Transformed x_local x:{Xlocal} y:{Ylocal} z:{Zlocal}" + CEND)
    #     rospy.loginfo(CRED2 + f"Correction Vector x:{self.gnc.correction_vector_g.position.x} y:{self.gnc.correction_vector_g.position.y} z:{self.gnc.correction_vector_g.position.z}" + CEND)
    #     rospy.loginfo(CRED2 + f"Local Offset x:{self.gnc.local_offset_pose_g.x} y:{self.gnc.local_offset_pose_g.y} z:{self.gnc.local_offset_pose_g.z}" + CEND)

    #     # Offset the coordinates by the initial position
    #     x = Xlocal + self.gnc.correction_vector_g.position.x + self.gnc.local_offset_pose_g.x
    #     y = Ylocal + self.gnc.correction_vector_g.position.y + self.gnc.local_offset_pose_g.y
    #     z = Zlocal + self.gnc.correction_vector_g.position.z + self.gnc.local_offset_pose_g.z

    #     rospy.loginfo(f"Destination set to x:{x} y:{y} z:{z} in origin frame")

    #     self.gnc.waypoint_g.pose.position = Point(x, y, z)
    #     self.gnc.local_pos_pub.publish(self.gnc.waypoint_g)

    # def publish_pose(self):
    #     rate = rospy.Rate(10)  # 10 Hz
    #     while not rospy.is_shutdown():
    #         current_pose = self.gnc.get_current_location()
    #         pose_msg = PoseStamped()
    #         pose_msg.header.stamp = rospy.Time.now()
    #         pose_msg.header.frame_id = "map"
    #         pose_msg.pose.position = current_pose
    #         pose_msg.pose.orientation = self.gnc.current_pose_g.pose.pose.orientation
    #         self.CorrectedPose_Pub.publish(pose_msg)
    #         rate.sleep()

if __name__ == '__main__':
    rospy.init_node('CorrectedPose_PublisherNode', anonymous=True)
    namespace = rospy.get_param('~namespace', 'pursuer')
    Offset_x = rospy.get_param('~offset_x', 0)
    Offset_y = rospy.get_param('~offset_y', 0)
    Offset_z = rospy.get_param('~offset_z', 0)
    Offset = Point(Offset_x,Offset_y,Offset_z)
    # rospy.loginfo(CBLUE + f"Using namespace: {namespace}"+ CEND)
    # rospy.loginfo(CRED + f"Using offset: ({Offset_x},{Offset_y},{Offset_z})"+ CEND)
    # rospy.loginfo("Starting pose publisher...")

    try:
        CorrectedPose_publisher = CorrectedPosePublisher(namespace,Offset)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
