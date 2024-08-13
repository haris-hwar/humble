#!/usr/bin/env python
import rospy
from PrintColours import CEND, CGREEN2
from py_gnc_functions import gnc_api

def main():
    rospy.init_node("drone_controller", anonymous=True)

    # Create objects for the API for the pursuer drone with its namespace
    pursuer_drone = gnc_api(namespace='/pursuer/')

    # Wait for FCU connection
    pursuer_drone.wait4connect()
    rospy.loginfo(CGREEN2 + "pursuer FCU connected" + CEND)

    # Create local reference frame
    # pursuer_drone.initialize_local_frame()
    # rospy.loginfo(CGREEN2 + "pursuer local reference frame initialized" + CEND)

    rate = rospy.Rate(3)

    # Specify the landing coordinates
    landing_coordinates = [0, 0, 0, 0]  # Modify this to your desired coordinates

    while not rospy.is_shutdown():
        # Set the destination to the landing coordinates
        pursuer_drone.set_destination(
            x=landing_coordinates[0], y=landing_coordinates[1], z=landing_coordinates[2], psi=landing_coordinates[3])
        rospy.loginfo(CGREEN2 + f"pursuer destination set to x:{landing_coordinates[0]} y:{landing_coordinates[1]} z:{landing_coordinates[2]} psi:{landing_coordinates[3]}" + CEND)

        if pursuer_drone.check_waypoint_reached():
            rospy.loginfo(CGREEN2 + "pursuer waypoint reached" + CEND)
            rospy.loginfo(CGREEN2 + "Initiating landing." + CEND)
            pursuer_drone.land()
            rospy.loginfo(CGREEN2 + "pursuer landing initiated" + CEND)
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
