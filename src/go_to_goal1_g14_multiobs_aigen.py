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
        rospy.init_node('go_to_goal', anonymous=True)

        self.turtle1_velocity_topic = '/turtlebot3/cmd_vel'
        self.relative_velocity = Twist()

        # Initialize turtle velocities
        self.turtle1_velocity = Twist()
        self.turtle_velocities = [Twist() for _ in range(4)]

        self.velocity_publisher_waffle = rospy.Publisher('/turtlebot3/cmd_vel', Twist, queue_size=10)
        self.velocity_publishers = [
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
        self.robot_poses = [Point() for _ in range(4)]
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

    def turtle1_velocity_callback(self, msg):
        self.turtle1_velocity = msg

    def turtle_velocity_callback(self, msg, index):
        self.turtle_velocities[index] = msg

    def calculate_relative_velocity(self):
        self.relative_velocity.linear.x = self.turtle1_velocity.linear.x - sum(t.linear.x for t in self.turtle_velocities) / 4
        self.relative_velocity.linear.y = self.turtle1_velocity.linear.y - sum(t.linear.y for t in self.turtle_velocities) / 4
        self.relative_velocity.angular.z = self.turtle1_velocity.angular.z - sum(t.angular.z for t in self.turtle_velocities) / 4

    def goal_movement(self):
        self.goal_pose.x = float(input("Enter goal_pose.x: "))
        self.goal_pose.y = float(input("Enter goal_pose.y: "))
        nr = 10
        delay_time = float(input("Enter the delay time in seconds before the robot starts moving: "))
        rospy.loginfo(f"Starting movement in {delay_time} seconds...")
        time.sleep(delay_time)
        while not rospy.is_shutdown():
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose_waffle.x)**2 + (self.goal_pose.y - self.robot_pose_waffle.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose_waffle.y, self.goal_pose.x - self.robot_pose_waffle.x)
            angle_difference = self.angle_to_goal - self.robot_pose_waffle.z

            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            for i in range(4):
                do = sqrt((self.robot_poses[i].x - self.robot_pose_waffle.x)**2 + (self.robot_poses[i].y - self.robot_pose_waffle.y)**2)
                relative_pos_vector = Vector3()
                relative_pos_vector.x = (self.robot_poses[i].x - self.robot_pose_waffle.x)
                relative_pos_vector.y = (self.robot_poses[i].y - self.robot_pose_waffle.y)
                relative_pos_vector.z = 0

                dmin = 0.350
                dmin_by_do = (dmin / do)
                lamdaa_radians = np.arcsin(dmin_by_do)
                lambdaa = math.degrees(lamdaa_radians)
                dot_product = (self.relative_velocity.linear.x * relative_pos_vector.x) + (self.relative_velocity.linear.y * relative_pos_vector.y)
                magnitude_rel_vel = sqrt((self.relative_velocity.linear.x**2) + (self.relative_velocity.linear.y**2))
                magnitude_rel_pos = sqrt((relative_pos_vector.x**2) + (relative_pos_vector.y**2))

                if magnitude_rel_vel == 0 or magnitude_rel_pos == 0:
                    thetaa = 0
                else:
                    thetaa = np.arccos(dot_product / (magnitude_rel_vel * magnitude_rel_pos))

                alphaa = math.degrees(thetaa)
                psi_waffle = self.robot_pose_waffle.z
                psi_waffle_deg = math.degrees(psi_waffle)

                alphaa_plus_lambdaa = alphaa + lambdaa
                alphaa_minus_lambdaa = alphaa - lambdaa

                linear_velocity = 0.5
                angular_velocity = self.angular_velocity_scale * angle_difference

                if (alphaa < lambdaa) and (do < nr):
                    if ((psi_waffle_deg) < (alphaa_plus_lambdaa)) and ((psi_waffle_deg) > (alphaa)):
                        self.vel_msg_waffle.linear.x = 0.5
                        self.vel_msg_waffle.angular.z = (angular_velocity - 3)
                        print("Steering left")
                    elif ((psi_waffle_deg) > (alphaa_minus_lambdaa)) and ((psi_waffle_deg) < (alphaa)):
                        self.vel_msg_waffle.linear.x = 0.5
                        self.vel_msg_waffle.angular.z = (angular_velocity + 3)
                        print("Steering right")
                elif self.distance_to_goal > self.goal_reached_threshold:
                    self.vel_msg_waffle.linear.x = 0.5
                    self.vel_msg_waffle.angular.z = self.angular_velocity_scale * angle_difference
                    print("Marching to goal")
                elif self.distance_to_goal <= self.goal_reached_threshold:
                    self.vel_msg_waffle.linear.x = 0
                    self.vel_msg_waffle.angular.z = 0
                    print("Goal reached")

                self.velocity_publishers[i].publish(self.vel_msgs[i])

            self.velocity_publisher_waffle.publish(self.vel_msg_waffle)

    def euler_from_quaternion(self, x, y, z, w):
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = asin(2 * (w * y - z * x))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

if __name__ == '__main__':
    try:
        turtlebot_gtg = TurtleBotGTG()
        turtlebot_gtg.goal_movement()
        rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
