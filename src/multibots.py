#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publish_velocity_command(velocity_cmd):
    # List of the topic names for the cmd_vel of each TurtleBot
    topics = [
        '/wheel1/cmd_vel',
        '/wheel2/cmd_vel',
        '/wheel3/cmd_vel',
        '/wheel4/cmd_vel'
    ]
    
    # Create a dictionary to hold the publishers
    publishers = {}
    
    # Initialize the publishers for each topic
    for topic in topics:
        publishers[topic] = rospy.Publisher(topic, Twist, queue_size=10)
    
    # Publish the velocity command to each topic
    for topic, publisher in publishers.items():
        publisher.publish(velocity_cmd)

if __name__ == '__main__':
    try:
        
        rospy.init_node('multi_turtlebot_velocity_publisher', anonymous=True)
        
        
        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0.5  
        velocity_cmd.angular.z = 0.25 
     
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            publish_velocity_command(velocity_cmd)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
