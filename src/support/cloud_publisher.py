#!/usr/bin/env python3

import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import os

def publish_pointcloud():
    rospy.init_node('pcd_publisher', anonymous=True)
    pub = rospy.Publisher('/pointcloud_pcd', PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    home_dir = os.path.expanduser('~/pointclouds')
    pcd_file_path = os.path.join(home_dir, 'mypointcloud5.pcd')

    cloud = pcl.load(pcd_file_path)

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()

    pointcloud_msg = pc2.create_cloud_xyz32(header, cloud.to_list())

    rospy.loginfo("Publishing point cloud from PCD file...")
    while not rospy.is_shutdown():
        pointcloud_msg.header.stamp = rospy.Time.now()
        pub.publish(pointcloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pointcloud()
    except rospy.ROSInterruptException:
        pass
