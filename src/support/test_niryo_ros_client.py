#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from grasp_n_sorter.srv import  jointsParm, jointsParmRequest
from geometry_msgs.msg import PointStamped


def get_grasp_pose():
    # Process your point cloud and determine the grasp pose
    j1, j2, j3 = 0.1, 0.1, 0.1
    j4, j5, j6 = 0.0, 0, 0.0
    return j1, j2, j3, j4, j5, j6

def request_grasp_pose():
    rospy.init_node('compute_grasp_pose_client')
    rospy.wait_for_service('/niryo_joints_service')
    try:
        grasp_pose_service = rospy.ServiceProxy('/niryo_joints_service', jointsParm)
        j1, j2, j3, j4, j5, j6 = get_grasp_pose()
        request = jointsParmRequest(j1, j2, j3, j4, j5, j6)
        response = grasp_pose_service(request)
        if response.success:
            rospy.loginfo("Successfully moved to grasp pose: %s", response.message)
        else:
            rospy.logwarn("Failed to move to grasp pose: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    request_grasp_pose()