#!/usr/bin/env python3
# Node for taking the PointCloud published in the topic '/stereo_publisher/stereo/points' published by DepthAI-ROS-nodes
# and saving in a file at home address

import rospy
import pcl
import os
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def pointcloud_callback(msg):
    # Convert ROS PointCloud2 message to PCL PointCloud
    pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    cloud = pcl.PointCloud()
    cloud.from_list(list(pc))

    # Expand the home directory
    home_dir = os.path.expanduser('~')
    save_path = os.path.join(home_dir, 'mypointcloud.pcd')

    # Save the PCL point cloud to a PCD file
    pcl.save(cloud, save_path)
    rospy.loginfo(f"PointCloud saved to {save_path}")
    rospy.signal_shutdown("PointCloud captured and saved")

if __name__ == '__main__':
    rospy.init_node('save_pointcloud', anonymous=True)
    pointcloud_topic = "/camera/depth/color/points"
    rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)
    rospy.loginfo("Waiting for a point cloud frame...")
    rospy.spin()
