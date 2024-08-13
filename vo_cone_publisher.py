#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import math
import pt_vo_check3 

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        
        rospy.Subscriber('/pursuer/corrected_pose', PoseStamped, self.pursuer_position_callback)
        rospy.Subscriber('/target/corrected_pose', PoseStamped, self.target_position_callback)
        
        self.pursuer_pos = None
        self.target_pos = None
        self.rate = rospy.Rate(2)  # 2 Hz

        # Create an instance of pt_vo_check3.DroneController
        self.pt_vo_check3_controller = pt_vo_check3.DroneController()

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        
        # rospy.loginfo("DroneController initialized and subscribers set up")

    def pursuer_position_callback(self, pose):
        self.pursuer_pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        # rospy.loginfo(f"Pursuer position updated: {self.pursuer_pos}")

    def target_position_callback(self, pose):
        self.target_pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        # rospy.loginfo(f"Target position updated: {self.target_pos}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(qx, qy, qz, qw)
 
    def euler_from_target(self):
        x, y, z = self.target_pos
        yaw = math.atan2(y, x)
        pitch = math.atan2(z, math.sqrt(x**2 + y**2))
        roll = 0
        return roll, pitch, yaw

    def create_cone_marker(self, frame_id, marker_id, position, orientation, length, radius):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cone"
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        n_segments = 36
        delta_theta = 2 * math.pi / n_segments

        for i in range(n_segments):
            theta = i * delta_theta
            p1 = Point(0, 0, 0)
            p2 = Point(length, radius * math.cos(theta), radius * math.sin(theta))
            p3 = Point(length, radius * math.cos(theta + delta_theta), radius * math.sin(theta + delta_theta))
            marker.points.extend([p3, p2, p1])

        marker.pose.position = position
        marker.pose.orientation = orientation
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0, 0, 0.25)
        return marker

    def run(self):
        # rospy.loginfo("DroneController is running")
        frame_id = "map"
        marker_id = 0
        orientation = self.quaternion_from_euler(0, 0, 0)

        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            if self.pursuer_pos and self.target_pos:
                position = Point(self.pursuer_pos[0], self.pursuer_pos[1], self.pursuer_pos[2])
                
                # Calculate doi and rpz as single float values
                doi = math.sqrt((self.pursuer_pos[0] - self.target_pos[0])**2 + 
                                (self.pursuer_pos[1] - self.target_pos[1])**2 +
                                (self.pursuer_pos[2] - self.target_pos[2])**2)
                rpz = 2.0
                
                length = self.pt_vo_check3_controller.calculate_dvo(doi, rpz)
                radius = self.pt_vo_check3_controller.calculate_rvo(doi, rpz)

                cone_marker = self.create_cone_marker(frame_id, marker_id, position, orientation, length, radius)
                self.marker_pub.publish(cone_marker)
                
            rate.sleep()

if __name__ == "__main__":
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
