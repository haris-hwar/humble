#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy
import math, time
import numpy as np
# Twist message
from geometry_msgs.msg import Twist, Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# Global parameters
x_cor = 10
y_cor = 0
yaw_in_radians = 0
yaw_rate = 0
flag = 0

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('pursuer/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('pursuer/mavros/cmd/takeoff', CommandTOL)
            takeoffService(altitude=10)
            print("Takeoff initiated")
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)

    def setArm(self):
        rospy.wait_for_service('pursuer/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('pursuer/mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service('pursuer/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('pursuer/mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def setStabilizedMode(self):
        rospy.wait_for_service('pursuer/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('pursuer/mavros/set_mode', SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set." % e)

    def setOffboardMode(self):
        rospy.wait_for_service('pursuer/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('pursuer/mavros/set_mode', SetMode)
            flightModeService(custom_mode='GUIDED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)

    def setAltitudeMode(self):
        rospy.wait_for_service('pursuer/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('pursuer/mavros/set_mode', SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set." % e)

    def setPositionMode(self):
        rospy.wait_for_service('pursuer/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('pursuer/mavros/set_mode', SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set." % e)

    def setAutoLandMode(self):
        rospy.wait_for_service('pursuer/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('pursuer/mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Autoland Mode could not be set." % e)

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        
        # Instantiate a Twist message
        self.twist = Twist()

        # A Message for the current local position of the drone
        self.local_pos = Point()

        # Circle parameters
        self.theta = np.linspace(0, 2 * np.pi, 100)
        self.r = np.sqrt(100)  # the radius of the circle
        self.x1 = self.r * np.cos(self.theta)
        self.x2 = self.r * np.sin(self.theta)
        self.abc = 0

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update velocity setpoint
    def updateVelocity(self):
        d = 0.5
        if -d <= self.threshold(self.local_pos.x, self.x1[self.abc]) <= d and -d <= self.threshold(self.local_pos.y, self.x2[self.abc]) <= d:
            self.abc += 1
            if self.abc >= len(self.x1):
                self.abc = 0

        self.twist.linear.x = (self.x1[self.abc] - self.local_pos.x) * 0.1  # proportional control for x-direction
        self.twist.linear.y = (self.x2[self.abc] - self.local_pos.y) * 0.1  # proportional control for y-direction
        self.twist.linear.z = (10 - self.local_pos.z) * 0.1  # maintaining constant altitude

    def threshold(self, x, y):
        return abs(x - y)

# Main function
def main():
    # initiate node
    rospy.init_node('circle_velocity_control', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('/pursuer/mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('/pursuer/mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Velocity setpoint publisher    
    sp_pub = rospy.Publisher('/pursuer/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    time.sleep(5)

    # Set in takeoff mode and takeoff to default altitude (10 m)
    if cnt.state.armed:
        modes.setTakeoff()
        time.sleep(9)

    # Activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateVelocity()
        sp_pub.publish(cnt.twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
