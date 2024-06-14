import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

def create_spherical_sector_marker(frame_id, r, h, a, phi_a, theta_a):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # Set the size of the points
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0  # Alpha
    marker.color.r = 1.0  # Red
    marker.color.g = 0.0  # Green
    marker.color.b = 0.0  # Blue

    alpha = math.atan(a / (r - h))

    for rho in [r * i / 10.0 for i in range(11)]:
        for phi in [phi_a - alpha / 2 + alpha * i / 10.0 for i in range(11)]:
            for theta in [theta_a - alpha / 2 + alpha * i / 10.0 for i in range(11)]:
                x = rho * math.cos(theta) * math.sin(phi)
                y = rho * math.sin(theta) * math.sin(phi)
                z = rho * math.cos(phi)
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                marker.points.append(p)
    
    return marker

if __name__ == '__main__':
    rospy.init_node('spherical_sector_marker')

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    r = 1.0  # Example radius
    h = 0.5  # Example height
    a = 0.5  # Example base radius
    phi_a = 0.5  # Example axis polar angle
    theta_a = 0.5  # Example axis azimuthal angle

    marker = create_spherical_sector_marker('base_link', r, h, a, phi_a, theta_a)
    
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rospy.sleep(0.1)

