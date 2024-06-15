#!/usr/bin/env python

import rospy
from grasp_n_sorter.srv import  jointsParm, jointsParmResponse
from niryo_one_python_api.niryo_one_api import NiryoOne

def handle_joints_pose(req):
    rospy.loginfo("Received Grasp Pose: j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f", req.j1, req.j2, req.j3, req.j4, req.j5, req.j6)
    n = NiryoOne()
    try:
        joints_array = [req.j1, req.j2, req.j3, req.j4, req.j5, req.j6]
        n.move_joints(joints_array)
        return jointsParmResponse(True, "Successfully moved to grasp pose")
    except Exception as e:
        rospy.logerr("Failed to move to grasp pose: %s", e)
        return jointsParmResponse(False, str(e))

def niryo_grasp_pose_server():
    rospy.init_node('niryo_grasp_pose_server')
    s = rospy.Service('niryo_grasp_pose', jointsParm, handle_joints_pose)
    rospy.loginfo("Ready to receive and execute grasp pose.")
    rospy.spin()

if __name__ == "__main__":
    niryo_grasp_pose_server()