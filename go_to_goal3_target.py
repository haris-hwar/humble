#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TransformPose_GAZ2MAV_Publisher:
    def __init__(self, namespace='target', offset=Point(5,0,0)):
        self.initialOffset = offset
        self.namespace = namespace
        self.transformed_yaw = math.pi / 2  # Example transformed yaw (90 degrees)

    def transform_goal(self, goal_pose):
        x = goal_pose.pose.position.x - self.initialOffset.x
        y = goal_pose.pose.position.y - self.initialOffset.y
        z = goal_pose.pose.position.z - self.initialOffset.z
        
        goal_orientation = goal_pose.pose.orientation
        goal_orientation_quaternion = (
            goal_orientation.x,
            goal_orientation.y,
            goal_orientation.z,
            goal_orientation.w
        )
        (goal_roll, goal_pitch, goal_yaw) = euler_from_quaternion(goal_orientation_quaternion)
        transformed_yaw = goal_yaw + self.transformed_yaw
        transformed_orientation_quaternion = quaternion_from_euler(goal_roll, goal_pitch, transformed_yaw)

        x_transformed = x * math.cos(transformed_yaw) - y * math.sin(transformed_yaw)
        y_transformed = x * math.sin(transformed_yaw) + y * math.cos(transformed_yaw)
        z_transformed = z
        
        transformed_goal_pose = PoseStamped()
        transformed_goal_pose.pose.position = Point(x_transformed, y_transformed, z_transformed)
        transformed_goal_pose.pose.orientation = Quaternion(
            x=transformed_orientation_quaternion[0],
            y=transformed_orientation_quaternion[1],
            z=transformed_orientation_quaternion[2],
            w=transformed_orientation_quaternion[3]
        )
        
        return transformed_goal_pose

class GoToGoal:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        
        # Publishers and Subscribers
        self.vel_pub = rospy.Publisher('/target/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/target/mavros/local_position/odom', Odometry, self.odom_callback)
        
        # Current position and orientation
        self.current_pose = None
        self.current_orientation = None

        # Goal point
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position = Point(10.0, -10.0, 5.0)  # Example goal point (x, y, z)
        self.goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # No rotation

        # TransformPose_GAZ2MAV_Publisher instance
        self.transformer = TransformPose_GAZ2MAV_Publisher()
        
        # Transform the goal point
        self.transformed_goal_pose = self.transformer.transform_goal(self.goal_pose)

    def odom_callback(self, data):
        self.current_pose = data.pose.pose.position
        self.current_orientation = data.pose.pose.orientation

    def calculate_distance_and_angles(self, goal, current_pose):
        dx = goal.position.x - current_pose.x
        dy = goal.position.y - current_pose.y
        dz = goal.position.z - current_pose.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        azimuth = math.atan2(dy, dx)
        elevation = math.atan2(dz, math.sqrt(dx**2 + dy**2))

        return distance, azimuth, elevation

    def go_to_goal(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue

            distance, azimuth, elevation = self.calculate_distance_and_angles(self.transformed_goal_pose.pose, self.current_pose)
            rospy.loginfo(f"Distance: {distance}, Azimuth: {azimuth}, Elevation: {elevation}")

                        # Set ROS parameters
            rospy.set_param('/target/azimuth', azimuth)
            rospy.set_param('/target/elevation', elevation)

            vel_msg = TwistStamped()
            vel_msg.header.stamp = rospy.Time.now()
            vel_msg.twist.linear.x = min(0.5, distance) * math.cos(azimuth) * math.cos(elevation)
            vel_msg.twist.linear.y = min(0.5, distance) * math.sin(azimuth) * math.cos(elevation)
            vel_msg.twist.linear.z = min(0.5, distance) * math.sin(elevation)

            self.vel_pub.publish(vel_msg)

            if distance < 0.1:  # Position tolerance
                rospy.loginfo("Goal reached")
                break

            rate.sleep()

if __name__ == '__main__':
    try:
        go_to_goal = GoToGoal()
        go_to_goal.go_to_goal()
    except rospy.ROSInterruptException:
        pass
