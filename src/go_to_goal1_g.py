#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians, degrees
import math
from math import asin ,acos


class TurtleBotGTG:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtlebot3/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/turtlebot3/odom', Odometry, self.get_turtlebot_pose)
        self.robot_burger_subscriber = rospy.Subscriber('/wheel/odom', Odometry, self.get_turtlebot_pose_burger)
        self.robot_pose = Point()
        self.robot_pose_burger = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.goal_reached_threshold = 0.1  
        self.angular_velocity_scale = 1

    def get_turtlebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose.z) = self.euler_from_quaternion(*quaternion)

    def get_turtlebot_pose_burger(self, data):
        self.robot_pose_burger.x = data.pose.pose.position.x
        self.robot_pose_burger.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_burger.z) = self.euler_from_quaternion(*quaternion)

    
    
    

    def vector_between_points(self,point, point2):
      
      return [point2.x - point.x, point2.y - point.y, point2.z - point.z]
    

    def calculate_relative_velocity(Vx1, Vy1, Vx2, Vy2):
        Vmag1 = math.sqrt(Vx1**2 + Vy1**2)
        theta1 = math.atan2(Vy1, Vx1)


        Vmag2 = math.sqrt(Vx2**2 + Vy2**2)
        theta2 = math.atan2(Vy2, Vx2)
        # Calculate relative velocity in the x-direction
        Vrel_x = Vmag1 * math.cos(theta1) - Vmag2 * math.cos(theta2)
    
        # Calculate relative velocity in the y-direction
        Vrel_y = Vmag1 * math.sin(theta1) - Vmag2 * math.sin(theta2)
    
        return Vrel_x, Vrel_y
    

    def angle_between_vectors(self, Vx1, Vy1, Vx2, Vy2):
        
        # Calculate the dot product
        dot_product = Vx1 * Vx2 + Vy1 * Vy2

        # Calculate the magnitudes
        magnitude_V1 = math.sqrt(Vx1**2 + Vy1**2)
        magnitude_V2 = math.sqrt(Vx2**2 + Vy2**2)

        # Calculate the cosine of the angle
        cosine_theta = dot_product / (magnitude_V1 * magnitude_V2)

        # Ensure the cosine value is within the valid range [-1, 1]
        cosine_theta = max(min(cosine_theta, 1.0), -1.0)

        # Use arccosine to find the angle in radians
        angle_radians = math.acos(cosine_theta)

        # Convert radians to degrees
        angle_degrees = math.degrees(angle_radians)

        return angle_degrees


    def goal_movement(self):
        
        self.goal_pose.x = float(input("Enter goal_pose.x: "))
        self.goal_pose.y = float(input("Enter goal_pose.y: "))
        

        while not rospy.is_shutdown():
            #K_linear = 0.5
            print('BURGER X:',format(self.robot_pose_burger.x),'BURGER Y:',format(self.robot_pose_burger.y),'BURGER Z:',format(self.robot_pose_burger.z))
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)

            angle_difference = self.angle_to_goal - self.robot_pose.z
            
            
            d_o= self.distance_to_goal = sqrt((self.robot_pose_burger.x - self.robot_pose.x)**2 + (self.robot_pose_burger.y - self.robot_pose.y)**2)
            
             
            relative_pos_vector=Point()
            relative_pos_vector.x=(self.robot_pose_burger.x - self.robot_pose.x)
            relative_pos_vector.y=(self.robot_pose_burger.y - self.robot_pose.y)
            relative_pos_vector.z=0


            Vmag1 = math.sqrt(self.robot_pose.x**2 + self.robot_pose.y**2)
            theta1 = math.atan2(self.robot_pose.y, self.robot_pose.x)


            Vmag2 = math.sqrt(self.robot_pose_burger.x**2 + self.robot_pose_burger.y**2)
            theta2 = math.atan2(self.robot_pose_burger.y, self.robot_pose_burger.x)
            # Calculate relative velocity in the x-direction
            Vrel_x = Vmag1 * math.cos(theta1) - Vmag2 * math.cos(theta2)
    
            # Calculate relative velocity in the y-direction
            Vrel_y = Vmag1 * math.sin(theta1) - Vmag2 * math.sin(theta2)

            relative_vel_vector=Point()
            relative_vel_vector.x=Vrel_x
            relative_vel_vector.y=Vrel_y 
            relative_vel_vector.z=0

            d_min = 0.350

            print("dmin:",d_min)
            print("do:",d_o)
            

            # Calculate the angle in radians
            angle_radians = asin(min(d_min / d_o, 1.0))

            # Convert radians to degrees if necessary
            lambdaa = math.degrees(angle_radians)

            #print("Angle in radians:", angle_radians)
            print("lambda::", lambdaa)

            
            #alpha = self.angle_between_vectors(relative_vel_vector.x,relative_vel_vector.y, relative_pos_vector.x, relative_pos_vector.y)
             

            # Calculate the dot product
            dot_product = relative_vel_vector.x * relative_vel_vector.y + relative_pos_vector.x* relative_pos_vector.y

            # Calculate the magnitudes
            magnitude_V1 = math.sqrt(relative_vel_vector.x**2 + relative_vel_vector.y **2)
            magnitude_V2 = math.sqrt(relative_pos_vector.x**2 + relative_pos_vector.y**2)

            # Calculate the cosine of the angle
            cosine_theta = dot_product / (magnitude_V1 * magnitude_V2)

            # Ensure the cosine value is within the valid range [-1, 1]
            cosine_theta = max(min(cosine_theta, 1.0), -1.0)

            # Use arccosine to find the angle in radians
            alpha_radians = math.acos(cosine_theta)

            # Convert radians to degrees
            alpha = math.degrees(alpha_radians) 


            


            print("alpha::", alpha)

            if alpha < lambdaa:
                print("possibility of collision")

            


            # Ensure angle difference is within the range of -pi to pi
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            linear_speed = self.distance_to_goal 
            angular_velocity = self.angular_velocity_scale * angle_difference

            # Set linear and angular velocities
            self.vel_msg.linear.x = 0.5
            self.vel_msg.angular.z = angular_velocity

            # Publish velocity command
            self.velocity_publisher.publish(self.vel_msg)

            # Check if goal is reached
            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Reached the goal!")
                break

    def euler_from_quaternion(self, x, y, z, w):
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = asin(2 * (w * y - z * x))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

if __name__ == '__main__':
    try:
        turtlebot_gtg = TurtleBotGTG()
        turtlebot_gtg.goal_movement()
    except rospy.ROSInterruptException:
        pass

