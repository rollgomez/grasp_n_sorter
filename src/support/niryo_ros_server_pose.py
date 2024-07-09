#!/usr/bin/env python
#CODE INSIDE THE RASPBERRY PI OF THE NIRYO
import rospy
from grasp_n_sorter.srv import  poseParm, poseParmResponse
from niryo_one_python_api.niryo_one_api import NiryoOne

def handle_pose(req):
    rospy.loginfo("Received Pose: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", req.x, req.y, req.z, req.roll, req.pitch, req.yaw)
    n = NiryoOne()
    try:
        pose_array = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
        n.set_arm_max_velocity(10)
        n.move_pose(pose_array)
        return poseParmResponse(True, "Successfully moved to pose")
    except Exception as e:
        rospy.logerr("Failed to move to pose: %s", e)
        return poseParmResponse(False, str(e))

def niryo_pose_server():
    rospy.init_node('niryo_pose_server')
    s = rospy.Service('niryo_pose_service', poseParm, handle_pose)
    rospy.loginfo("Ready to receive and execute pose.")
    rospy.spin()

if __name__ == "__main__":
    niryo_pose_server()