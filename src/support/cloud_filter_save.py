#!/usr/bin/env python3

import rospy
import pcl
import os
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def filter_pointcloud(pointcloud):
    # Define the workspace limits
    x_min, x_max = -0.10, 0.10
    y_min, y_max = -0.11, 0.05
    z_min, z_max = 0.40, 0.58

    # Before
    # x_min, x_max = -0.10, 0.10
    # y_min, y_max = -0.05, 0.05
    # z_min, z_max = 0.45, 0.60

    # Extract points within the specified workspace
    filtered_points = []
    for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point[0], point[1], point[2]
        if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
            filtered_points.append(point)

    # Create a new PCL PointCloud
    cloud = pcl.PointCloud()
    cloud.from_list(filtered_points)

    return cloud

def pointcloud_callback(msg):
    filtered_cloud = filter_pointcloud(msg)

    # Expand the home directory and set the save path
    home_dir = os.path.expanduser('~')
    save_path = os.path.join(home_dir, 'filtered_pointcloud.pcd')

    # Save the filtered point cloud to a PCD file
    pcl.save(filtered_cloud, save_path)
    rospy.loginfo(f"Filtered point cloud saved to {save_path}")
    rospy.signal_shutdown("Filtered point cloud captured and saved")

if __name__ == '__main__':
    rospy.init_node('filter_save_pointcloud', anonymous=True)
    pointcloud_topic = "/pointcloud_pcd"  # Replace with your point cloud topic
    rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)
    rospy.loginfo("Waiting for a point cloud frame...")
    rospy.spin()
