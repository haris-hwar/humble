#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import Sphere, Cylinder

def compute_intersection():
    # Define parameters of the sphere and cylinder
    sphere_center = ShapelyPoint(0, 0, 0)
    sphere_radius = 1.0
    cylinder_center = ShapelyPoint(0, 0, 0)
    cylinder_radius = 0.5
    cylinder_height = 2.0

    # Create sphere and cylinder objects using shapely
    sphere = Sphere(sphere_center, sphere_radius)
    cylinder = Cylinder(cylinder_center, cylinder_radius, cylinder_height)

    # Compute intersection geometry
    intersection = sphere.intersection(cylinder)

    return intersection

def visualize_intersection():
    rospy.init_node('intersection_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('intersection_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Compute intersection geometry
    intersection = compute_intersection()

    # Publish intersection geometry
    intersection_marker = Marker()
    intersection_marker.header.frame_id = "map"
    intersection_marker.type = Marker.MESH_RESOURCE
    intersection_marker.action = Marker.ADD
    intersection_marker.scale.x = 1.0
    intersection_marker.scale.y = 1.0
    intersection_marker.scale.z = 1.0
    intersection_marker.color.a = 0.5  # Set transparency
    intersection_marker.color.r = 1.0  # Set color (red)
    intersection_marker.mesh_resource = "package://rviz_visualization/intersection_mesh.stl"  # Replace with actual mesh file
    intersection_marker.mesh_use_embedded_materials = True

    # Publish the intersection marker
    while not rospy.is_shutdown():
        intersection_marker.header.stamp = rospy.Time.now()
        marker_pub.publish(intersection_marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        visualize_intersection()
    except rospy.ROSInterruptException:
        pass
