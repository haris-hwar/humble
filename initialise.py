#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import time

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)

        self.target_state = State()
        self.target_pose = PoseStamped()
        self.pursuer_state = State()
        self.pursuer_pose = PoseStamped()

        agent_name = rospy.get_namespace()
        print(agent_name, '+++++++++++++>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>#########################################')

        # Subscribe to the MAVROS state and local position topics for both UAVs
        rospy.Subscriber('/$agent_name$/mavros/state', State, self.target_state_callback)
        rospy.Subscriber('/target/mavros/local_position/pose', PoseStamped, self.target_pose_callback)
        rospy.Subscriber('/pursuer/mavros/state', State, self.pursuer_state_callback)
        rospy.Subscriber('/pursuer/mavros/local_position/pose', PoseStamped, self.pursuer_pose_callback)

        # Create services for arming and setting mode for both UAVs
        self.target_arming_client = rospy.ServiceProxy('/target/mavros/cmd/arming', CommandBool)
        self.target_set_mode_client = rospy.ServiceProxy('/target/mavros/set_mode', SetMode)
        self.pursuer_arming_client = rospy.ServiceProxy('/pursuer/mavros/cmd/arming', CommandBool)
        self.pursuer_set_mode_client = rospy.ServiceProxy('/pursuer/mavros/set_mode', SetMode)

        # Create publishers for the local position goal for both UAVs
        self.target_pose_goal_pub = rospy.Publisher('/target/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pursuer_pose_goal_pub = rospy.Publisher('/pursuer/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    def target_state_callback(self, state):
        self.target_state = state

    def target_pose_callback(self, pose):
        self.target_pose = pose

    def pursuer_state_callback(self, state):
        self.pursuer_state = state

    def pursuer_pose_callback(self, pose):
        self.pursuer_pose = pose

    def arm(self, vehicle):
        if vehicle == "target":
            rospy.wait_for_service('/target/mavros/cmd/arming')
            try:
                self.set_mode("target", 'GUIDED')
                self.target_arming_client(True)
                rospy.loginfo("Target drone armed")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
        elif vehicle == "pursuer":
            rospy.wait_for_service('/pursuer/mavros/cmd/arming')
            try:
                self.set_mode("pursuer", 'GUIDED')
                self.pursuer_arming_client(True)
                rospy.loginfo("Pursuer drone armed")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def set_mode(self, vehicle, mode):
        if vehicle == "target":
            rospy.wait_for_service('/target/mavros/set_mode')
            try:
                self.target_set_mode_client(0, mode)
                rospy.loginfo("Target mode set to %s", mode)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
        elif vehicle == "pursuer":
            rospy.wait_for_service('/pursuer/mavros/set_mode')
            try:
                self.pursuer_set_mode_client(0, mode)
                rospy.loginfo("Pursuer mode set to %s", mode)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def takeoff(self, vehicle):
        if vehicle == "target":
            rospy.wait_for_service('/target/mavros/cmd/takeoff')
            try:
                takeoff = rospy.ServiceProxy('/target/mavros/cmd/takeoff', CommandTOL)
                response = takeoff(altitude=10.0)  # Set the desired takeoff altitude
                rospy.loginfo("Takeoff command sent. Response: %s", response)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
        elif vehicle == "pursuer":
            rospy.wait_for_service('/pursuer/mavros/cmd/takeoff')
            try:
                takeoff_Pursuer = rospy.ServiceProxy('/pursuer/mavros/cmd/takeoff', CommandTOL)
                response = takeoff_Pursuer(altitude=10.0)  # Set the desired takeoff altitude
                rospy.loginfo("Takeoff command sent to Pursuer. Response: %s", response)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed for Pursuer: %s", e)

if __name__ == "__main__":
    try:
        controller = DroneController()

        # Arm the drones
        controller.arm("target")
        controller.arm("pursuer")

        # Change the mode to GUIDED
        controller.set_mode("target", 'GUIDED')
        controller.set_mode("pursuer", 'GUIDED')

        # Take off
        controller.takeoff("target")
        controller.takeoff("pursuer")

        rospy.loginfo("----------Hovering----------")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
