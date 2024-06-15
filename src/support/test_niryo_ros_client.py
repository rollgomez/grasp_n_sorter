#!/usr/bin/env python3

import rospy
from grasp_n_sorter.srv import  jointsParm, jointsParmRequest

def get_grasp_pose():
    # Process your point cloud and determine the grasp pose
    j1, j2, j3 = 0.1, 0.2, 0.3
    j4, j5, j6 = 0.0, 0.57, 0.0
    return j1, j2, j3, j4, j5, j6

def request_grasp_pose():
    rospy.init_node('compute_grasp_pose_client')
    rospy.wait_for_service('niryo_grasp_pose')
    try:
        grasp_pose_service = rospy.ServiceProxy('niryo_grasp_pose', jointsParm)
        j1, j2, j3, j4, j5, j6= get_grasp_pose()
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