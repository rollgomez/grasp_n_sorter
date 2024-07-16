#!/usr/bin/env python
#CODE INSIDE THE RASPBERRY PI OF THE NIRYO
import rospy
from grasp_n_sorter.srv import  jointsParm, jointsParmResponse
from niryo_one_python_api.niryo_one_api import NiryoOne

def handle_joints(req):
    rospy.loginfo("Received joints: j1=%f, j2=%f, j3=%f, j4=%f, j5=%f, j6=%f", req.j1, req.j2, req.j3, req.j4, req.j5, req.j6)
    n = NiryoOne()
    try:
        joints_array = [req.j1, req.j2, req.j3, req.j4, req.j5, req.j6]
        n.set_arm_max_velocity(10)
        n.move_joints(joints_array)
        return jointsParmResponse(True, "Successfully moved joints")
    except Exception as e:
        rospy.logerr("Failed to move joints: %s", e)
        return jointsParmResponse(False, str(e))

def niryo_joints_server():
    rospy.init_node('niryo_joints_server')
    s = rospy.Service('niryo_joints_service', jointsParm, handle_joints)
    rospy.loginfo("Ready to receive and execute joints.")
    rospy.spin()

if __name__ == "__main__":
    niryo_joints_server()