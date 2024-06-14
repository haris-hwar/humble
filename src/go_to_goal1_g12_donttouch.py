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
        
        self.velocity_publisher_waffle = rospy.Publisher('/turtlebot3/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_obstacle = rospy.Publisher('/wheel/cmd_vel', Twist, queue_size=10)
        self.robot_waffle_subscriber = rospy.Subscriber('/turtlebot3/odom', Odometry, self.get_turtlebot_pose_waffle)
        self.robot_obstacle_subscriber = rospy.Subscriber('/wheel/odom', Odometry, self.get_turtlebot_pose_obstacle)
        rospy.Subscriber(self.turtle1_velocity_topic, Twist, self.turtle1_velocity_callback)
        rospy.Subscriber(self.turtle2_velocity_topic, Twist, self.turtle2_velocity_callback)
        self.robot_pose_waffle = Point()
        self.robot_pose_obstacle = Point()
        self.goal_pose = Point()
        self.goal_pose_1 = Point()
        self.vel_msg_waffle = Twist()
        self.vel_msg_obstacle = Twist()
        self.distance_to_goal = 0.0
        self.distance_to_goal_1 = 0.0
        self.angle_to_goal = 0.0
        self.angle_to_goal_1 = 0.0
        self.goal_reached_threshold = 0.25
        self.goal_reached_threshold_1 = 0.25
        self.angular_velocity_scale = 0.5
        self.angular_velocity_scale_1 = 0.1
        #self.initial_yaw_obstacle = None

    def get_turtlebot_pose_waffle(self, data):
        self.robot_pose_waffle.x = data.pose.pose.position.x
        self.robot_pose_waffle.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_waffle.z) = self.euler_from_quaternion(*quaternion)

    def get_turtlebot_pose_obstacle(self, data):
        self.robot_pose_obstacle.x = data.pose.pose.position.x
        self.robot_pose_obstacle.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_obstacle.z) = self.euler_from_quaternion(*quaternion)

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
        #global Vrel_x,Vrel_y,Vrel_z
        #Vrel_x = self.relative_velocity.linear.x
        #Vrel_y = self.relative_velocity.linear.y
        #Vrel_z = self.relative_velocity.angular.z


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

    def goal_movement(self):
        # Function to move the goal
        self.goal_pose.x = 10
        self.goal_pose.y = 0
        self.goal_pose_1.x = 5
        self.goal_pose_1.y = -5      
        #self.goal_pose.x = float(input("Enter goal_pose.x: "))
        #self.goal_pose.y = float(input("Enter goal_pose.y: "))
        #self.goal_pose_1.x = float(input("Enter goal_pose_1.x: "))
        #self.goal_pose_1.y = float(input("Enter goal_pose_1.y: "))
        nr=10
        #desired_orientation = 0.0

        while not rospy.is_shutdown():
            #self.vel_msg_obstacle.linear.x = 0.5 #read this
            #self.vel_msg_obstacle.angular.z = 0   #read this
            
            #self.velocity_publisher_obstacle.publish(self.vel_msg_obstacle)

            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose_waffle.x)**2 + (self.goal_pose.y - self.robot_pose_waffle.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose_waffle.y, self.goal_pose.x - self.robot_pose_waffle.x)
            angle_difference = self.angle_to_goal - self.robot_pose_waffle.z

            self.distance_to_goal_1 = sqrt((self.goal_pose_1.x - self.robot_pose_obstacle.x)**2 + (self.goal_pose_1.y - self.robot_pose_obstacle.y)**2)
            self.angle_to_goal_1 = atan2(self.goal_pose_1.y - self.robot_pose_obstacle.y, self.goal_pose_1.x - self.robot_pose_obstacle.x)
            angle_difference_1 = self.angle_to_goal_1 - self.robot_pose_obstacle.z

            # Ensure angle difference is within the range of -pi to pi
            if angle_difference_1 > math.pi:
                angle_difference_1 -= 2 * math.pi
            elif angle_difference_1 < -math.pi:
                angle_difference_1 += 2 * math.pi 

            
            # Adjust angular velocity for straight line movement
            angular_velocity_1 = self.angular_velocity_scale_1 * angle_difference_1

            self.vel_msg_obstacle.linear.x = 0.5  # Adjust linear velocity as needed
            self.vel_msg_obstacle.angular.z = angular_velocity_1
            self.velocity_publisher_obstacle.publish(self.vel_msg_obstacle)


            # Distance between waffle-agent and the obstacle
            do = sqrt(((self.robot_pose_obstacle.x - self.robot_pose_waffle.x)**2) + ((self.robot_pose_obstacle.y - self.robot_pose_waffle.y)**2))

            relative_pos_vector= Vector3()
            relative_pos_vector.x=(self.robot_pose_obstacle.x - self.robot_pose_waffle.x)
            relative_pos_vector.y=(self.robot_pose_obstacle.y - self.robot_pose_waffle.y)
            relative_pos_vector.z=0
           
            # Radius of the obstacle
            dmin = 0.500

            # Calculate Vrel_x
            Vx = self.turtle1_velocity.linear.x
            
            
            theta2 = self.robot_pose_waffle.z

            Vrel_x = Vx  * math.cos(theta2)

            # Calculate Vrel_y
            Vrel_y = Vx *  math.sin(theta2)

            # Assign relative velocities
            self.relative_velocity.linear.x = Vrel_x
            self.relative_velocity.linear.y = Vrel_y

            self.relative_velocity.angular.z = self.turtle1_velocity.angular.z - self.turtle2_velocity.angular.z
            #print("self.relative_velocity.linear.x:", self.relative_velocity.linear.x)
            #print("self.relative_velocity.linear.y:", self.relative_velocity.linear.y)
            #print("self.relative_velocity.angular.z:", self.relative_velocity.angular.z)

            #print("dmin:",dmin)
            print("do:",do)  
            print("nr: ",nr) 
            if do==0:
                dmin_by_do=0

            else:
                dmin_by_do=(dmin / do)
            #print("dmin_do",dmin_by_do)

            # Calculate the angle in radians
            lamdaa_radians = np.arcsin(dmin_by_do)

            #print("lamdaa_radians:", lamdaa_radians)

            # Convert radians to degrees if necessary
            lambdaa = math.degrees(lamdaa_radians)
            #print("lambda::", lambdaa)

            # Dot product of relative velocity vector and relative position vector
            dot_product = (self.relative_velocity.linear.x * relative_pos_vector.x) + (self.relative_velocity.linear.y * relative_pos_vector.y)
            #dot_product = (Vrel_x * relative_pos_vector.x) + (Vrel_y * relative_pos_vector.y)
            # Magnitude of relative velocity vector
            magnitude_rel_vel = sqrt((self.relative_velocity.linear.x**2) + (self.relative_velocity.linear.y**2))

            # Magnitude of relative position vector
            magnitude_rel_pos = sqrt((relative_pos_vector.x**2) + (relative_pos_vector.y**2))

            # Angle between relative velocity vector and relative position vector
            #alphaa_radians = np.arccos(dot_product / (magnitude_rel_vel * magnitude_rel_pos))
            #cos_theta = dot_product / (magnitude_rel_vel * magnitude_rel_pos)
            
            if magnitude_rel_vel == 0 or magnitude_rel_pos == 0:
                print("One of the magnitudes is zero. Cannot compute cosine theta.")
                thetaa = 0  # Set a default value or handle it as required
            else:
                # Calculate the cosine of the angle
                thetaa = np.arccos(dot_product / (magnitude_rel_vel * magnitude_rel_pos))

            #print("thetaa_rad:", thetaa)

            # Calculate alpha_radians safely
            #alphaa_radians = np.arccos(cos_theta)

            alphaa = math.degrees(thetaa)
            #print("alphaa:", alphaa)

            # Heading of the waffle
            psi_waffle = self.robot_pose_waffle.z   
            #print("psi_waffle:", psi_waffle)
            psi_waffle_deg = math.degrees(psi_waffle)
            #print("psi_waffle_degrees:", psi_waffle_deg)

            # Alphaa + lambdaa and Alphaa - lambdaa
            alphaa_plus_lambdaa = alphaa + lambdaa
            alphaa_minus_lambdaa = alphaa - lambdaa

            # Ensure angle difference is within the range of -pi to pi
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            # Commands for linear and angluar velocity
            linear_velocity = 0.5
            angular_velocity = self.angular_velocity_scale * angle_difference
            #print("angular velocity: ", angular_velocity)
            # Enters into collision avoidance maneuver when alphaa < lambdaa, because the relative velocity lies in the collision cone

            print("Linear velocity: ", self.turtle1_velocity.linear.x)
            print("Angular velocity: ", self.turtle1_velocity.angular.z)

            if (alphaa < lambdaa) and (do<nr):  
                # alphaa - angle between relative velocity vector and relative position vector
                # is less than lambdaa - half angle of the collision cone
                # and do - distance between waffle and obstacle less than neighbouring region
                
   
            # Steer commands based on alphaa, lambdaa and psi_waffle
                if ((psi_waffle_deg) < (alphaa_plus_lambdaa)) and ((psi_waffle_deg) > (alphaa)):
                    # steer to the left
                    self.vel_msg_waffle.linear.x=0.5
                    self.vel_msg_waffle.angular.z=(angular_velocity-3)
                    #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                    print("Steering left")
                    print(("do: ",do),("nr: ",nr),("alpha: ",alphaa),("lambdaa: ",lambdaa))

                elif ((psi_waffle_deg) > (alphaa_minus_lambdaa)) and ((psi_waffle_deg) < (alphaa)):
                    # steer to the right
                     self.vel_msg_waffle.linear.x=0.5
                     self.vel_msg_waffle.angular.z=((angular_velocity)+3)
                     #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                     print("Steering right")
                     
                
            elif self.distance_to_goal > self.goal_reached_threshold:
                    self.vel_msg_waffle.linear.x= 0.5
                    self.vel_msg_waffle.angular.z=self.angular_velocity_scale * angle_difference
                    #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                    print("marching to goal")

                
            elif self.distance_to_goal <= self.goal_reached_threshold:
                    self.vel_msg_waffle.linear.x=0
                    self.vel_msg_waffle.angular.z=0
                    #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                    print("Goal reached")

            #self.vel_msg_waffle.linear.x =  0.5
            #self.vel_msg_waffle.angular.z=angular_velocity
            #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)

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
