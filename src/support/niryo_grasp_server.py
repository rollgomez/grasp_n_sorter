#!/usr/bin/env python
import rospy
import time
import math
from grasp_n_sorter.srv import graspObject, graspObjectResponse
from niryo_one_python_api.niryo_one_api import *

def handle_grasp(req):
    rospy.loginfo("Received grasp request: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
    n = NiryoOne()
    try:
        # Approach the object
        approach_pose = [req.x + 0.05 , req.y, req.z + 0.1, req.roll, req.pitch, req.yaw]
        n.set_arm_max_velocity(30)
        n.move_pose(*approach_pose)
        rospy.sleep(1)

        # Open gripper 
        n.change_tool(TOOL_GRIPPER_2_ID)
        n.open_gripper(TOOL_GRIPPER_2_ID,200)

        # Adjust the gripper's orientation
        n.set_arm_max_velocity(10)
        n.move_pose(req.x + 0.025, req.y, req.z + 0.05, req.roll, req.pitch, req.yaw)
        rospy.sleep(1)

        # Grasp the object
        n.move_pose(req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
        rospy.sleep(1)
        n.close_gripper(TOOL_GRIPPER_2_ID,200)

        return graspObjectResponse(True, "Successfully grasped object")
    except Exception as e:
        rospy.logerr("Failed to grasp object: %s", e)
        return graspObjectResponse(False, str(e))

def niryo_grasp_server():
    rospy.init_node('niryo_grasp_server')
    s = rospy.Service('niryo_grasp_service', graspObject, handle_grasp)
    rospy.loginfo("Ready to receive and execute grasp commands.")
    rospy.spin()

if __name__ == "__main__":
    niryo_grasp_server()

