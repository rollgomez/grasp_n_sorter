#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as point_cloud2

# Global variable to store the latest point cloud
latest_cloud = None

# Callback function to update the latest point cloud
def cloudCallback(msg):
    global latest_cloud
    latest_cloud = msg

# Function to publish the latest point cloud at regular intervals
def publish_cloud(pub):
    global latest_cloud
    if latest_cloud is not None:
        # Update the header timestamp
        latest_cloud.header.stamp = rospy.Time.now()
        pub.publish(latest_cloud)

def main():
    rospy.init_node('cloud_publisher_rviz')

    # Subscribe to the filtered point cloud topic
    rospy.Subscriber('filtered_cloud', PointCloud2, cloudCallback)

    # Publisher for the repeated point cloud
    pub = rospy.Publisher('cloud_pcd', PointCloud2, queue_size=1)

    # Set the publishing rate (e.g., 10 Hz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_cloud(pub)
        rate.sleep()

if __name__ == '__main__':
    main()