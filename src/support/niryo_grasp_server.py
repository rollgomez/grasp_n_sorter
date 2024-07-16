#!/usr/bin/env python
import rospy
import time
import math
from grasp_n_sorter.srv import graspObject, graspObjectResponse
from niryo_one_python_api.niryo_one_api import *
import numpy as np

def handle_grasp(req):
    rospy.loginfo("Received grasp request: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f, approach vector: (%f, %f, %f)", 
                  req.x, req.y, req.z, req.roll, req.pitch, req.yaw, req.approach.x, req.approach.y, req.approach.z)
    n = NiryoOne()
    try:
        # Define the grasp point and orientation
        grasp_point = np.array([req.x, req.y, req.z])
        grasp_orientation = [req.roll, req.pitch, req.yaw]

        # Define the approach vector from the request
        approach_vector = np.array([req.approach.x, req.approach.y, req.approach.z])
        approach_vector = approach_vector / np.linalg.norm(approach_vector)  # Normalize the approach vector

        # Calculate the intermediate point (5 cm away from the grasp point in the direction of the approach vector)
        distance_away = 0.05  # 5 cm away
        intermediate_point = grasp_point + (approach_vector * distance_away)

        # Move to the intermediate point
        n.set_arm_max_velocity(30)
        n.move_pose(intermediate_point[0], intermediate_point[1], intermediate_point[2],
                    grasp_orientation[0], grasp_orientation[1], grasp_orientation[2])
        rospy.sleep(1)

        # Open gripper 
        n.change_tool(TOOL_GRIPPER_2_ID)
        n.open_gripper(TOOL_GRIPPER_2_ID, 200)

        # Move to the final grasp point
        n.set_arm_max_velocity(10)
        n.move_pose(grasp_point[0], grasp_point[1], grasp_point[2],
                    grasp_orientation[0], grasp_orientation[1], grasp_orientation[2])
        rospy.sleep(1)

        # Grasp the object
        n.close_gripper(TOOL_GRIPPER_2_ID, 200)

        return graspObjectResponse(True, "Successfully grasped object")
    except Exception as e:
        rospy.logerr("Failed to grasp object: %s", str(e))
        return graspObjectResponse(False, str(e))

def niryo_grasp_server():
    rospy.init_node('niryo_grasp_server')
    s = rospy.Service('niryo_grasp_service', graspObject, handle_grasp)
    rospy.loginfo("Ready to receive and execute grasp commands.")
    rospy.spin()

if __name__ == "__main__":
    niryo_grasp_server()
