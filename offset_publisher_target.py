#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

class PoseCorrector:
    def __init__(self):
        rospy.init_node('initial_pose_offset', anonymous=True)

        # Initial position in Gazebo (x, y, z)
        self.initial_position = Vector3()
        self.initial_position.x = 5.0 
        self.initial_position.y = 5.0
        self.initial_position.z = 0.0

        self.initial_yaw =  -1.56 # in radians
        self.initial_pitch = 0      # in radians
        self.initial_roll = 0       # in radians

        # Convert initial Euler angles to quaternion
        self.initial_orientation_q = quaternion_from_euler(self.initial_roll, self.initial_pitch, self.initial_yaw)

        # Subscribe to the current pose topic
        self.pose_subscriber = rospy.Subscriber('/target/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Publisher for the corrected pose
        self.corrected_pose_publisher = rospy.Publisher('/target/corrected_pose', PoseStamped, queue_size=10)

        self.current_offset = Vector3()
        self.current_pose = PoseStamped()

    def pose_callback(self, data):
        self.current_pose = data
        self.update_corrected_pose()

    def update_corrected_pose(self):
        # Correct the pose based on the initial position and the offset
        corrected_pose = PoseStamped()
        corrected_pose.header = self.current_pose.header
        corrected_pose.pose.position.x = self.current_pose.pose.position.x + self.current_offset.x + self.initial_position.x
        corrected_pose.pose.position.y = self.current_pose.pose.position.y + self.current_offset.y + self.initial_position.y
        corrected_pose.pose.position.z = self.current_pose.pose.position.z + self.current_offset.z + self.initial_position.z

        # Get current orientation as a quaternion
        current_orientation_q = self.current_pose.pose.orientation
        current_orientation_list = [current_orientation_q.x, current_orientation_q.y, current_orientation_q.z, current_orientation_q.w]

        # Multiply the current orientation quaternion with the initial orientation quaternion
        corrected_orientation_q = quaternion_multiply(current_orientation_list, self.initial_orientation_q)
        
        corrected_pose.pose.orientation.x = corrected_orientation_q[0]
        corrected_pose.pose.orientation.y = corrected_orientation_q[1]
        corrected_pose.pose.orientation.z = corrected_orientation_q[2]
        corrected_pose.pose.orientation.w = corrected_orientation_q[3]

        # Convert quaternion to roll, pitch, and yaw for logging
        (corrected_roll, corrected_pitch, corrected_yaw) = euler_from_quaternion(corrected_orientation_q)

        # Publish the corrected pose
        self.corrected_pose_publisher.publish(corrected_pose)

        # Log the corrected pose and orientation (yaw, pitch, roll)
        # rospy.loginfo("Corrected Pose: x: %f, y: %f, z: %f", corrected_pose.pose.position.x, corrected_pose.pose.position.y, corrected_pose.pose.position.z)
        # rospy.loginfo("Corrected Orientation: roll: %f, pitch: %f, yaw: %f", corrected_roll, corrected_pitch, corrected_yaw)

if __name__ == '__main__':
    try:
        pose_corrector = PoseCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
