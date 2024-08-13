#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import cos, sin, pi
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CorrectedPosePublisher:
    def __init__(self, namespace='pursuer', Offset=Point(0, 0, 0)):
        self.initialOffset = Offset
        self.CorrectedPose_Pub = rospy.Publisher(f'/{namespace}/corrected_pose', PoseStamped, queue_size=10)
        self.localPos_sub = rospy.Subscriber(f'/{namespace}/mavros/global_position/local', Odometry, self.localPos_Callback)

    def localPos_Callback(self, msg):
        CorrectedPose_msg = self.ComputeCorrectedPose(msg)
        CorrectedPose_msg.header.stamp = rospy.Time.now()
        CorrectedPose_msg.header.frame_id = "map"
        self.CorrectedPose_Pub.publish(CorrectedPose_msg)

    def ComputeCorrectedPose(self, msg):
        CurrentLocalPos = msg.pose.pose.position
        x = CurrentLocalPos.x
        y = CurrentLocalPos.y
        z = CurrentLocalPos.z
        
        CurrentLocalOrientation = msg.pose.pose.orientation
        CurrentLocalOrientation_quaternion = (
            CurrentLocalOrientation.x,
            CurrentLocalOrientation.y,
            CurrentLocalOrientation.z,
            CurrentLocalOrientation.w
        )
        (CurrentLocalRoll, CurrentLocalPitch, CurrentLocalYaw) = euler_from_quaternion(CurrentLocalOrientation_quaternion)
        transformed_yaw = CurrentLocalYaw - (pi/2)
        TransformedLocalOrientation_quaternion = quaternion_from_euler(CurrentLocalRoll, CurrentLocalPitch, transformed_yaw)

        x_transformed = x * cos(pi/2) + y * sin(pi/2)
        y_transformed = -1 * x * sin(pi/2) + y * cos(pi/2)
        z_transformed = z

        x_transformed += self.initialOffset.x
        y_transformed += self.initialOffset.y
        z_transformed += self.initialOffset.z

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

class CorrectedOdometryPublisher:
    def __init__(self, namespace='pursuer', Offset=Point(0, 0, 0)):
        self.initialOffset = Offset
        self.CorrectedOdometry_Pub = rospy.Publisher(f'/{namespace}/corrected_odometry', Odometry, queue_size=10)
        self.localOdom_sub = rospy.Subscriber(f'/{namespace}/mavros/local_position/odom', Odometry, self.localOdom_Callback)

    def localOdom_Callback(self, msg):
        CorrectedOdometry_msg = self.ComputeCorrectedOdometry(msg)
        CorrectedOdometry_msg.header.stamp = rospy.Time.now()
        CorrectedOdometry_msg.header.frame_id = "map"
        self.CorrectedOdometry_Pub.publish(CorrectedOdometry_msg)

    def ComputeCorrectedOdometry(self, msg):
        CurrentLocalPos = msg.pose.pose.position
        x = CurrentLocalPos.x
        y = CurrentLocalPos.y
        z = CurrentLocalPos.z

        CurrentLocalOrientation = msg.pose.pose.orientation
        CurrentLocalOrientation_quaternion = (
            CurrentLocalOrientation.x,
            CurrentLocalOrientation.y,
            CurrentLocalOrientation.z,
            CurrentLocalOrientation.w
        )
        (CurrentLocalRoll, CurrentLocalPitch, CurrentLocalYaw) = euler_from_quaternion(CurrentLocalOrientation_quaternion)
        transformed_yaw = CurrentLocalYaw - (pi/2)
        TransformedLocalOrientation_quaternion = quaternion_from_euler(CurrentLocalRoll, CurrentLocalPitch, transformed_yaw)

        x_transformed = x * cos(pi/2) + y * sin(pi/2)
        y_transformed = -1 * x * sin(pi/2) + y * cos(pi/2)
        z_transformed = z

        x_transformed += self.initialOffset.x
        y_transformed += self.initialOffset.y
        z_transformed += self.initialOffset.z

        CurrentLocalPos_Transformed = Point(x_transformed, y_transformed, z_transformed)

        # Transform velocity
        CurrentLocalVel = msg.twist.twist.linear
        vel_x = CurrentLocalVel.x
        vel_y = CurrentLocalVel.y
        vel_z = CurrentLocalVel.z

        vel_x_transformed = vel_x * cos(pi/2) + vel_y * sin(pi/2)
        vel_y_transformed = -1 * vel_x * sin(pi/2) + vel_y * cos(pi/2)
        vel_z_transformed = vel_z

        # vel_x_transformed += self.initialOffset.x
        # vel_y_transformed += self.initialOffset.y
        # vel_z_transformed += self.initialOffset.z

        CorrectedOdometry = Odometry()
        CorrectedOdometry.header = msg.header
        CorrectedOdometry.child_frame_id = msg.child_frame_id
        CorrectedOdometry.pose.pose.position = CurrentLocalPos_Transformed
        CorrectedOdometry.pose.pose.orientation = Quaternion(
            x=TransformedLocalOrientation_quaternion[0],
            y=TransformedLocalOrientation_quaternion[1],
            z=TransformedLocalOrientation_quaternion[2],
            w=TransformedLocalOrientation_quaternion[3]
        )

        CorrectedOdometry.twist.twist.linear.x = vel_x_transformed
        CorrectedOdometry.twist.twist.linear.y = vel_y_transformed
        CorrectedOdometry.twist.twist.linear.z = vel_z_transformed
        CorrectedOdometry.twist.twist.angular = msg.twist.twist.angular  

        return CorrectedOdometry

if __name__ == '__main__':
    rospy.init_node('CorrectedPublisherNode', anonymous=True)
    namespace = rospy.get_param('~namespace', 'pursuer')
    Offset_x = rospy.get_param('~offset_x', 0)
    Offset_y = rospy.get_param('~offset_y', 0)
    Offset_z = rospy.get_param('~offset_z', 0)
    Offset = Point(Offset_x, Offset_y, Offset_z)

    try:
        CorrectedPose_publisher = CorrectedPosePublisher(namespace, Offset)
        CorrectedOdometry_publisher = CorrectedOdometryPublisher(namespace, Offset)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
