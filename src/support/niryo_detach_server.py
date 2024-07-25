#!/usr/bin/env python
#CODE INSIDE THE RASPBERRY PI OF THE NIRYO
import rospy
from grasp_n_sorter.srv import  poseParm, poseParmResponse
from niryo_one_python_api.niryo_one_api import NiryoOne

def handle_detach(req):
    rospy.loginfo("Received Pose: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", req.x_p, req.y_p, req.z_p, req.roll, req.pitch, req.yaw)
    n = NiryoOne()
    try:
        pose_array = [req.x_p, req.y_p, req.z_p + 0.05, req.roll, req.pitch, req.yaw]
        n.set_arm_max_velocity(30)
        n.move_pose(*pose_array)
        pose_array = [req.x_p, req.y_p, req.z_p, req.roll, req.pitch, req.yaw]
        n.set_arm_max_velocity(15)
        n.move_pose(*pose_array)
        n.change_tool(TOOL_GRIPPER_2_ID)
        n.open_gripper(TOOL_GRIPPER_2_ID, 200)
        rospy.sleep(1)
        n.close_gripper(TOOL_GRIPPER_2_ID, 200)

        return poseParmResponse(True, "Successfully left object")
    except Exception as e:
        rospy.logerr("Failed ;eave object: %s", e)
        return poseParmResponse(False, str(e))

def niryo_detach_server():
    rospy.init_node('niryo_detach_server')
    s = rospy.Service('niryo_detach_service', poseParm, handle_detach)
    rospy.loginfo("Ready to receive and execute pose to detach.")
    rospy.spin()

if __name__ == "__main__":
    niryo_detach_server()