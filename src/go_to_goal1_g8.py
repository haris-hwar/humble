#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion ,Vector3
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians, degrees
import math
from math import asin ,acos
import numpy as np

class TurtleBotGTG:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
         
        self.turtle1_velocity_topic = '/turtlebot3/cmd_vel'
        self.turtle2_velocity_topic = '/wheel/cmd_vel'
        self.relative_velocity = Twist()

        # Initialize turtle velocities
        self.turtle1_velocity = Twist()
        self.turtle2_velocity = Twist()

        self.velocity_publisher = rospy.Publisher('/turtlebot3/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_burger = rospy.Publisher('/wheel/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/turtlebot3/odom', Odometry, self.get_turtlebot_pose)
        self.robot_burger_subscriber = rospy.Subscriber('/wheel/odom', Odometry, self.get_turtlebot_pose_burger)
        rospy.Subscriber(self.turtle1_velocity_topic, Twist, self.turtle1_velocity_callback)
        rospy.Subscriber(self.turtle2_velocity_topic, Twist, self.turtle2_velocity_callback)
        self.robot_pose = Point()
        self.robot_pose_burger = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.vel_msg_burger = Twist()
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

    
    def turtle1_velocity_callback(self, msg):
        # Callback function for TurtleBot 1's velocity
        self.turtle1_velocity = msg

    def turtle2_velocity_callback(self, msg):
        # Callback function for TurtleBot 2's velocity
        self.turtle2_velocity = msg

    def calculate_relative_velocity(self):
        # Calculate the relative velocity between TurtleBot 1 and TurtleBot 2
        self.relative_velocity.linear.x = self.turtle1_velocity.linear.x - self.turtle2_velocity.linear.x
        self.relative_velocity.linear.y = self.turtle1_velocity.linear.y - self.turtle2_velocity.linear.y
        self.relative_velocity.angular.z = self.turtle1_velocity.angular.z - self.turtle2_velocity.angular.z
        print("self.relative_velocity.linear.x:", self.relative_velocity.linear.x)
        print("self.relative_velocity.linear.y:", self.relative_velocity.linear.y)
        print("self.relative_velocity.angular.z:", self.relative_velocity.angular.z)
    

    def vector_between_points(self,point, point2):
      
      return [point2.x - point.x, point2.y - point.y, point2.z - point.z]
    

    def calculate_relative_velocity_dummy(Vx1, Vy1, Vx2, Vy2):
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

            self.vel_msg_burger.linear.x = 0.5
            self.vel_msg_burger.angular.z = 0
            self.velocity_publisher_burger.publish(self.vel_msg_burger)
            #K_linear = 0.5
            print('BURGER X:',format(self.robot_pose_burger.x),'BURGER Y:',format(self.robot_pose_burger.y),'BURGER Z:',format(self.robot_pose_burger.z))
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)

            angle_difference = self.angle_to_goal - self.robot_pose.z
            
            
            d_o= self.distance_to_goal = sqrt((self.robot_pose_burger.x - self.robot_pose.x)**2 + (self.robot_pose_burger.y - self.robot_pose.y)**2)
            
             
            

            #relative_pos_vector= Vector3()
            #relative_pos_vector.x=(self.robot_pose.x - self.robot_pose_burger.x)
            #relative_pos_vector.y=(self.robot_pose.y - self.robot_pose_burger.y)
            #relative_pos_vector.z=0
            
            relative_pos_vector= Vector3()
            relative_pos_vector.x=(self.robot_pose_burger.x - self.robot_pose.x)
            relative_pos_vector.y=(self.robot_pose_burger.y - self.robot_pose.y)
            relative_pos_vector.z=0

            # Calculate Vrel_x
            Vx = self.turtle2_velocity.linear.x
            
            
            theta2 = self.robot_pose_burger.z

            Vrel_x = Vx  * math.cos(theta2)

            # Calculate Vrel_y
            Vrel_y = Vx *  math.sin(theta2)

            # Assign relative velocities
            self.relative_velocity.linear.x = Vrel_x
            self.relative_velocity.linear.y = Vrel_y


            

            # Calculate the relative velocity between TurtleBot 1 and TurtleBot 2
            #self.relative_velocity.linear.x = self.turtle1_velocity.linear.x - self.turtle2_velocity.linear.x
            #self.relative_velocity.linear.y = self.turtle1_velocity.linear.y - self.turtle2_velocity.linear.y
            self.relative_velocity.angular.z = self.turtle1_velocity.angular.z - self.turtle2_velocity.angular.z
            print("self.relative_velocity.linear.x:", self.relative_velocity.linear.x)
            print("self.relative_velocity.linear.y:", self.relative_velocity.linear.y)
            print("self.relative_velocity.angular.z:", self.relative_velocity.angular.z)
    
            

            

            d_min = 0.350
            d_min = 1

            print("dmin:",d_min)
            print("do:",d_o)
            

            # Calculate the angle in radians
            #angle_radians = asin(min(d_min / d_o))
            lamdaa_radians = np.arcsin(d_min / d_o)
            # Convert radians to degrees if necessary
            lambdaa = math.degrees(lamdaa_radians)

            #print("Angle in radians:", angle_radians)
            print("lambda::", lambdaa)

            
            #alpha = self.angle_between_vectors(relative_vel_vector.x,relative_vel_vector.y, relative_pos_vector.x, relative_pos_vector.y)
             

            # Calculate the dot product
            #dot_product = (self.relative_velocity.linear.x  * self.relative_velocity.linear.y) + (relative_pos_vector.x* relative_pos_vector.y)
            dot_product = (self.relative_velocity.linear.x  * relative_pos_vector.x) + (self.relative_velocity.linear.y* relative_pos_vector.y)
            print("dot_product:", dot_product)
            # Calculate the magnitudes
            magnitude_V1 = math.sqrt((self.relative_velocity.linear.x **2) + (self.relative_velocity.linear.y **2))
            print("magnitude_V1:", magnitude_V1)
            magnitude_V2 = math.sqrt((relative_pos_vector.x**2) + (relative_pos_vector.y**2))
            print("magnitude_V2:", magnitude_V2)

             # Check if any magnitude is zero
            if magnitude_V1 == 0 or magnitude_V2 == 0:
                print("One of the magnitudes is zero. Cannot compute cosine theta.")
                cosine_theta = 0  # Set a default value or handle it as required
            else:
                # Calculate the cosine of the angle
                cosine_theta = dot_product / (magnitude_V1 * magnitude_V2)

            print("cosine_theta_rad:", cosine_theta)

            # Calculate the cosine of the angle
            ##print("cosine_theta_rad:", cosine_theta)

            # Ensure the cosine value is within the valid range [-1, 1]
            #cosine_theta = max(min(cosine_theta, 1.0), -1.0)

            # Use arccosine to find the angle in radians
            alpha_radians = np.arccos(cosine_theta)


            # Convert radians to degrees
            alpha = math.degrees(alpha_radians) 
            turtle_heading_degrees = math.degrees(self.turtle1_velocity.angular.z)

            alpha_plus_lambdaa_degree = math.degrees(alpha+lambdaa)
            alpha_minus_lambdaa_degree = math.degrees(alpha-lambdaa)

            


            print("alpha::", alpha)

            

            


            # Ensure angle difference is within the range of -pi to pi
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            linear_speed = self.distance_to_goal 
            angular_velocity = self.angular_velocity_scale * angle_difference
            

            #if alpha < lambdaa:
            if (alpha_radians < lamdaa_radians) and (d_o < d_min):

                #new_angle=(lambdaa-alpha)
                print("possibility of collision")
                if ((turtle_heading_degrees) < (alpha_plus_lambdaa_degree)) and ((turtle_heading_degrees) > (alpha)):
                    

                    # Set linear and angular velocities
                    self.vel_msg.linear.x = 0.5
                    self.vel_msg.angular.z = 0.5

                elif ((turtle_heading_degrees) > (alpha_minus_lambdaa_degree)) and ((turtle_heading_degrees) < (alpha)):

                    # Set linear and angular velocities
                    self.vel_msg.linear.x = 0.5
                    self.vel_msg.angular.z = -0.5

                    #if vel_angle > math.pi:
                    
                        #vel_angle -= 2 * math.pi

                    #elif vel_angle < -math.pi:

                        #vel_angle += 2 * math.pi

                    #self.vel_msg.angular.z = vel_angle

                # Publish velocity command
                self.velocity_publisher.publish(self.vel_msg_burger)
            else:
                print("no collision")
                # Set linear and angular velocities
                self.vel_msg.linear.x = 0.5
                self.vel_msg.angular.z = 0.0

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
        rospy.sleep(0.5)
        
           

           
        
    except rospy.ROSInterruptException:
        pass

