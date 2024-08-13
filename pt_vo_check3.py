#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)

        rospy.Subscriber('/pursuer/corrected_pose', PoseStamped, self.pursuer_position_callback)
        rospy.Subscriber('/target/corrected_pose', PoseStamped, self.target_position_callback)
        rospy.Subscriber('/pursuer/corrected_odometry', TwistStamped, self.pursuer_velocity_callback)
        rospy.Subscriber('/target/corrected_odometry', TwistStamped, self.target_velocity_callback)

        self.pursuer_pos = None
        self.target_pos = None
        self.pursuer_vel = None
        self.target_vel = None    

        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("DroneController initialized and subscribers set up")

    def pursuer_position_callback(self, pose):
        self.pursuer_pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        # rospy.loginfo(f"Pursuer position updated: {self.pursuer_pos}")

    def target_position_callback(self, pose):
        self.target_pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        # rospy.loginfo(f"Target position updated: {self.target_pos}")

    def pursuer_velocity_callback(self, vel):
        self.pursuer_vel = [vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z]
        # rospy.loginfo(f"Pursuer velocity updated: {self.pursuer_vel}")

    def target_velocity_callback(self, vel):
        self.target_vel = [vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z]
        # rospy.loginfo(f"Target velocity updated: {self.target_vel}")

    def calculate_los_angles(self):
        if self.target_pos is None or self.pursuer_pos is None:
            rospy.logwarn("Position data for target or pursuer not yet received.")
            return None, None, None

        dx = self.target_pos[0] - self.pursuer_pos[0]
        dy = self.target_pos[1] - self.pursuer_pos[1]
        dz = self.target_pos[2] - self.pursuer_pos[2]
        
        los_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        los_theta_rad = math.atan2(dy, dx)
        los_theta_deg = math.degrees(los_theta_rad) % 360
        
        if los_theta_deg < 0:
            los_theta_deg += 360
        
        try:
            los_psi_rad = math.asin(dz / los_dist)
            los_psi_deg = math.degrees(los_psi_rad)
            
            if los_psi_deg < 0:
                los_psi_deg += 360
        except ZeroDivisionError:
            los_psi_deg = 0
        
        return los_dist, los_theta_rad, los_psi_rad

    def calculate_doi(self, los_dist):
        return los_dist

    def calculate_rpz(self):
        return 4.0

    def calculate_dvo(self, doi, rpz):
        return (doi**2 - rpz**2) / doi

    def calculate_rvo(self, doi, rpz):
        if doi <= rpz:
            rospy.logerr("in collision")
            return 0  # or handle appropriately
        return rpz * ((math.sqrt(doi**2 - rpz**2)) / doi)

    def calculate_alpha_vo(self, rvo, dvo):
        return math.degrees(math.atan2(rvo, dvo))

    def calculate_dvo_vector(self, los_theta_rad, los_psi_rad, dvo):
        dvo_vector_x = math.cos(los_theta_rad) * math.cos(los_psi_rad) * dvo
        dvo_vector_y = math.cos(los_theta_rad) * math.sin(los_psi_rad) * dvo
        dvo_vector_z = math.sin(los_theta_rad) * dvo

        return Vector3(dvo_vector_x, dvo_vector_y, dvo_vector_z)
    
    def calculate_relative_velocity(self):
        azimuth_pursuer = rospy.get_param('/pursuer/azimuth', None)
        elevation_pursuer = rospy.get_param('/pursuer/elevation', None)
        if azimuth_pursuer is None or elevation_pursuer is None:
            rospy.logwarn("Azimuth or elevation parameter not found for pursuer")
            return None
        
        azimuth_target = rospy.get_param('/target/azimuth', None)
        elevation_target = rospy.get_param('/target/elevation', None)
        if azimuth_target is None or elevation_target is None:
            rospy.logwarn("Azimuth or elevation parameter not found for target")
            return None
        

        vx_pursuer = 0.5 * math.cos(elevation_pursuer) * math.cos(azimuth_pursuer)
        vy_pursuer = 0.5 * math.cos(elevation_pursuer) * math.sin(azimuth_pursuer)
        vz_pursuer = 0.5 * math.sin(elevation_pursuer)

        vx_target = 0.5 * math.cos(elevation_target) * math.cos(azimuth_target)
        vy_target = 0.5 * math.cos(elevation_target) * math.sin(azimuth_target)
        vz_target = 0.5 * math.sin(elevation_target)

        Vrel = Vector3()

        Vrel.x = vx_pursuer - vx_target
        Vrel.y = vy_pursuer - vy_target
        Vrel.z = vz_pursuer - vz_target

        return Vrel

    def angle_difference_vectors(self, Vrel, dvo_vector, dvo):
        dot_product = (Vrel.x * dvo_vector.x) + (Vrel.y * dvo_vector.y) + (Vrel.z * dvo_vector.z)
        mag_Vrel = math.sqrt(Vrel.x**2 + Vrel.y**2 + Vrel.z**2)
        mag_dvo_vector = math.sqrt(dvo_vector.x**2 + dvo_vector.y**2 + dvo_vector.z**2)

        angle_diff_vectors = dot_product / (mag_Vrel * dvo)

        return dot_product, mag_Vrel, angle_diff_vectors, mag_dvo_vector 
    
    def collision_condition(self, angle_diff_vectors, alpha_vo, doi, dvo):
        if angle_diff_vectors > math.cos(math.radians(alpha_vo)) and doi < dvo:
            pass
            # rospy.loginfo("In collision course, inclusion of pursuer's velocity vector in VO")
        else:
            # rospy.loginfo("Not in collision course")
            pass
        
    def run(self):
        rospy.loginfo("DroneController is running")
        while not rospy.is_shutdown():
            if self.pursuer_pos and self.target_pos:
                los_dist, los_theta_rad, los_psi_rad = self.calculate_los_angles()
                rospy.loginfo(f"LOS angles calculated: dist: {los_dist}, theta_rad: {los_theta_rad}, psi_rad: {los_psi_rad}")

                if los_dist is not None:
                    doi = self.calculate_doi(los_dist)
                    rpz = self.calculate_rpz()
                    dvo = self.calculate_dvo(doi, rpz)
                    rvo = self.calculate_rvo(doi, rpz)
                    alpha_vo = self.calculate_alpha_vo(rvo, dvo)
                    
                    dvo_vector = self.calculate_dvo_vector(los_theta_rad, los_psi_rad, dvo)

                    Vrel = self.calculate_relative_velocity()
                    dot_product, mag_Vrel, angle_diff_vectors, mag_dvo_vector = self.angle_difference_vectors(Vrel, dvo_vector, dvo)
                    self.collision_condition(angle_diff_vectors, alpha_vo, doi, dvo)

                    rospy.loginfo(f"doi: {doi}, rpz: {rpz}, dvo: {dvo}, rvo: {rvo}, alpha_vo: {alpha_vo}")
                    rospy.loginfo(f"dot_product: {dot_product}, mag_Vrel: {mag_Vrel}, angle_diff_vectors: {angle_diff_vectors}, mag_dvo_vector: {mag_dvo_vector}")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
