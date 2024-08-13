#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from PrintColours import *
from math import radians, cos, sin, pi, degrees, sqrt
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

class TransformPose_GAZ2MAV_Publisher:
    def __init__(self, namespace='pursuer', offset=Point(0,0,0)):
        self.initialOffset = offset
        self.namespace = namespace
        self.CorrectedPose_Pub = rospy.Publisher(f'/{namespace}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.CurrentLocalPose_Transformed = PoseStamped()  # To store transformed pose
        self.transformed_yaw = 0  # Initialize transformed yaw
        self.goal = PoseStamped()  # To store the goal
        self.goal_received = False  # Flag to check if goal is received

    def CorrectAndPublishGoal(self, msg):
        CorrectedPose_msg = self.ComputeCorrectedPose(msg)
        CorrectedPose_msg.header.stamp = rospy.Time.now()
        CorrectedPose_msg.header.frame_id = "map"
        self.CorrectedPose_Pub.publish(CorrectedPose_msg)
        if self.goal_received:
            self.go_to_goal()

    def ComputeCorrectedPose(self, msg):
        CurrentLocalPos = msg.position
        x = CurrentLocalPos.x - self.initialOffset.x
        y = CurrentLocalPos.y - self.initialOffset.y
        z = CurrentLocalPos.z - self.initialOffset.z
        
        CurrentLocalOrientation = msg.orientation
        CurrentLocalOrientation_quaternion = (
            CurrentLocalOrientation.x,
            CurrentLocalOrientation.y,
            CurrentLocalOrientation.z,
            CurrentLocalOrientation.w
        )
        (CurrentLocalRoll, CurrentLocalPitch, CurrentLocalYaw) = euler_from_quaternion(CurrentLocalOrientation_quaternion)
        self.transformed_yaw = CurrentLocalYaw + (pi/2)
        TransformedLocalOrientation_quaternion = quaternion_from_euler(CurrentLocalRoll, CurrentLocalPitch, self.transformed_yaw)

        x_transformed = x * cos(self.transformed_yaw) - y * sin(self.transformed_yaw)
        y_transformed = x * sin(self.transformed_yaw) + y * cos(self.transformed_yaw)
        z_transformed = z
        
        CurrentLocalPos_Transformed = Point(x_transformed, y_transformed, z_transformed)
        CurrentLocalOrientation_Transformed = TransformedLocalOrientation_quaternion

        self.CurrentLocalPose_Transformed.pose.position = CurrentLocalPos_Transformed
        self.CurrentLocalPose_Transformed.pose.orientation = Quaternion(
            x=CurrentLocalOrientation_Transformed[0],
            y=CurrentLocalOrientation_Transformed[1],
            z=CurrentLocalOrientation_Transformed[2],
            w=CurrentLocalOrientation_Transformed[3]
        )

        return self.CurrentLocalPose_Transformed


    def publish_goal(self, goal_x, goal_y, goal_z, goal_roll, goal_pitch, goal_yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = Point(goal_x, goal_y, goal_z)
        goal_orientation_quat = quaternion_from_euler(goal_roll, goal_pitch, goal_yaw)
        goal_msg.pose.orientation = Quaternion(
            x=goal_orientation_quat[0],
            y=goal_orientation_quat[1],
            z=goal_orientation_quat[2],
            w=goal_orientation_quat[3]
        )
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(CBLUE + f"Published goal: ({goal_x}, {goal_y}, {goal_z}) with orientation ({goal_roll}, {goal_pitch}, {goal_yaw})" + CEND)

if __name__ == '__main__':
    rospy.init_node('CorrectedPose_PublisherNode', anonymous=True)
    namespace = rospy.get_param('~namespace', 'pursuer')
    offset_x = rospy.get_param('~offset_x', 0)
    offset_y = rospy.get_param('~offset_y', 0)
    offset_z = rospy.get_param('~offset_z', 0)
    Offset = Point(offset_x, offset_y, offset_z)
    rospy.loginfo(CBLUE + f"Using namespace: {namespace}" + CEND)
    rospy.loginfo(CRED + f"Using offset: ({offset_x},{offset_y},{offset_z})" + CEND)

    try:
        CorrectedPose_publisher = TransformPose_GAZ2MAV_Publisher(namespace, Offset)
        
        # Get the goal coordinates and orientation from the user
        goal_x = float(input("Enter the goal x-coordinate: "))
        goal_y = float(input("Enter the goal y-coordinate: "))
        goal_z = float(input("Enter the goal z-coordinate: "))

        # goal_roll = radians(0)   # example roll in radians
        # goal_pitch = radians(0)  # example pitch in radians
        # goal_yaw = 0 # example yaw in radians

        GazeboDestination = Pose()
        GazeboDestination.position = Point(goal_x,goal_y,goal_z)
        # GazeboDestination.orientation = Quaternion(*quaternion_from_euler(goal_roll, goal_pitch, goal_yaw))
        CorrectedPose_publisher.CorrectAndPublishGoal(GazeboDestination)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
