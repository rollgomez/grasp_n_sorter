#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3Stamped, Vector3
from sensor_msgs.msg import JointState
from grasp_n_sorter.srv import reqGrasp, reqGraspResponse, reqGraspRequest
from grasp_n_sorter.srv import  jointsParm, jointsParmRequest
from grasp_n_sorter.srv import  poseParm, poseParmRequest
from grasp_n_sorter.srv import graspObject, graspObjectRequest
from grasp_n_sorter.srv import classifyImg, classifyImgResponse, classifyImgRequest
from gpd.msg import GraspConfigList
from ikpy.chain import Chain

def move_robot_to_pose(x, y, z, roll, pitch, yaw):
    rospy.wait_for_service('/niryo_pose_service')
    try:
        niryo_pose_service = rospy.ServiceProxy('/niryo_pose_service', poseParm)
        request = poseParmRequest(x, y, z, roll, pitch, yaw)
        response = niryo_pose_service(request)
        if response.success:
            rospy.loginfo("Successfully moved to pose: %s", response.message)
            return response.success
        else:
            rospy.logwarn("Failed to move to pose: %s", response.message)
            return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False
    
def classify_object():
    rospy.wait_for_service('/classify_image')
    try:
        classify_image_service = rospy.ServiceProxy('/classify_image', classifyImg)
        confirmation = 1
        request = classifyImgRequest(confirmation)
        response = classify_image_service(request)
        return response.class_name, response.confidence_level
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return response.class_name, response.confidence_level

def leave_object(x, y, z, roll, pitch, yaw):
    rospy.wait_for_service('/niryo_detach_service')
    try:
        niryo_detach_service = rospy.ServiceProxy('/niryo_detach_service', poseParm)
        request = poseParmRequest(x, y, z, roll, pitch, yaw)
        response = niryo_detach_service(request)
        if response.success:
            rospy.loginfo("Successfully left object: %s", response.message)
            return response.success
        else:
            rospy.logwarn("Failed to leave object: %s", response.message)
            return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False
    
if __name__ == "__main__":
    rospy.init_node('test_model_node')
    confidence = 0
    roll = 0
    count = 0
    while confidence < 0.6:
        confirmation = leave_object(0.1, 0.15, 0.15, 0, 1.57, 0) #Move to camera
        # rospy.sleep(1)
        # if confirmation:
        #     class_name, confidence = classify_object()
        #     rospy.loginfo(class_name)
        #     rospy.loginfo(confidence)
        #     if roll < 1.57:
        #         roll+=1.57
        #     else:
        #         roll=-1.57
        # count+=1
        # if count == 6:
        #     class_name = 'codo'
        #     break
        confidence = 1