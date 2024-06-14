#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians, degrees
import math
from math import asin, acos
import numpy as np

class TurtleBotGTG:
    def __init__(self):
        rospy.init_node("go to goal", anonymous = True)
        self.turtle1_velocity_topic = '/turtlebot3/cmd_vel'
        self.turtle_obs_velocity_topic = 'wheel{i+1}/cmd_vel'

        # Initialize turtle velocities
        self.turtle1_velocity = Twist()
        self.turtle_obs_velocities = [Twist() for _ in range(4)]

        self.velocity_publisher_waffle = rospy.Publisher('/turtlebot3/cmd_vel', Twist, queue_size=10)
        self.velocity_publishers_obs = [
            rospy.Publisher(f'/wheel{i+1}/cmd_vel', Twist, queue_size=10) for i in range(4)
        ]

        self.robot_waffle_subscriber = rospy.Subscriber('/turtlebot3/odom', Odometry, self.get_turtlebot_pose_waffle)
        self.robot_subscribers = [
            rospy.Subscriber(f'/wheel{i+1}/odom', Odometry, self.get_turtlebot_pose_obstacle, callback_args=i) for i in range(4)
        ]

        rospy.Subscriber(self.turtle1_velocity_topic, Twist, self.turtle1_velocity_callback)
        for i in range(4):
            rospy.Subscriber(f'/wheel{i+1}/cmd_vel', Twist, self.turtle_velocity_callback, callback_args=i)

        self.robot_pose_waffle = Point()
        self.robot_poses_obs = [Point() for _ in range(4)]
        self.goal_pose = Point()
        self.vel_msg_waffle = Twist()
        self.vel_msgs = [Twist() for _ in range(4)]
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.goal_reached_threshold = 0.25
        self.angular_velocity_scale = 0.5     

    def get_turtlebot_pose_waffle(self, data):
        self.robot_pose_waffle.x = data.pose.pose.position.x
        self.robot_pose_waffle.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_waffle.z) = self.euler_from_quaternion(*quaternion)

    def get_turtlebot_pose_obstacle(self, data, index):
        self.robot_poses[index].x = data.pose.pose.position.x
        self.robot_poses[index].y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_poses[index].z) = self.euler_from_quaternion(*quaternion)
        self.calculate_distances()
    
    def turtle1_velocity_callback(self, msg):
        self.turtle1_velocity = msg 

    def turtle_velocity_callback(self, msg, index):
        self.turtle_obs_velocities[index] = msg

    def calculate_relative_velocity(self):
        self.relative_velocity_waffle = Twist()
        self.relative_velocity_waffle.linear.x = self.turtle1_velocity.linear.x - self.vel_msgs[0].linear.x
        self.relative_velocity_waffle.linear.y = self.turtle1_velocity.linear.y - self.vel_msgs[0].linear.y
        self.relative_velocity_waffle.angular.z = self.turtle1_velocity.angular.z - self.vel_msgs[0].angular.z
        for i in range(1, 4):
            self.relative_velocity_obs[i-1] = Twist()
            self.relative_velocity_obs[i-1].linear.x = self.turtle_obs_velocities[i].linear.x - self.vel_msgs[i].linear.x
            self.relative_velocity_obs[i-1].linear.y = self.turtle_obs_velocities[i].linear.y - self.vel_msgs[i].linear.y
            self.relative_velocity_obs[i-1].angular.z = self.turtle_obs_velocities[i].angular.z - self.vel_msgs[i].angular.z
            return self.relative_velocity_waffle, self.relative_velocity_obs
        
        return self.relative_velocity_waffle, self.relative_velocity_obs
    def calculate_distance_to_goal(self):
        self.distance_to_goal = math.sqrt((self.goal_pose.x - self.robot_pose_waffle.x)**2 + (self.goal_pose.y - self.robot_pose_waffle.y)**2)
        return self.distance_to_goal
    
    def calculate_angle_to_goal(self):
        self.angle_to_goal = math.atan2(self.goal_pose.y - self.robot_pose_waffle.y, self.goal_pose.x - self.robot_pose_waffle.x)
        return self.angle_to_goal
    
    def angle_difference(self):
        angle_difference = self.angle_to_goal - self.robot_pose_waffle.z

        if angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

    def calculate_distances(self):
        distances = []
        for i, obs_pose in enumerate(self.robot_poses_obs):
            distance = math.sqrt(
                (obs_pose.x - self.robot_pose_waffle.x)**2 +
                (obs_pose.y - self.robot_pose_waffle.y)**2
            )
            distances.append(distance)
            rospy.loginfo(f"Distance to obstacle {i+1}: {distance}")     

      



    

    def goal_movement(self):
        self.goal_pose.x = float(input("Enter goal_pose.x: "))
        self.goal_pose.y = float(input("Enter goal_pose.y: "))
        #neighbouring region
        nr = 10
        while not rospy.is_shutdown():
            pass