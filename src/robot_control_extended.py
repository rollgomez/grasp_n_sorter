#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from grasp_n_sorter.srv import reqGrasp, reqGraspResponse, reqGraspRequest
from grasp_n_sorter.srv import  jointsParm, jointsParmRequest
from grasp_n_sorter.srv import  poseParm, poseParmRequest
from grasp_n_sorter.srv import graspObject, graspObjectRequest
from grasp_n_sorter.srv import classifyImg, classifyImgResponse, classifyImgRequest
from gpd.msg import GraspConfigList
from ikpy.chain import Chain


def get_grasps(confirmation):
    rospy.wait_for_service('select_grasp_service')
    try: 
        get_grasp_client = rospy.ServiceProxy('select_grasp_service', reqGrasp)
        
        response = get_grasp_client(confirmation)
        return response.grasp_configs
    
    except rospy.ServiceException as error:
        rospy.loginfo(error)

def transform_point(point_camera, source_frame, target_frame, tf_buffer):
    point_in_camera = PointStamped()
    point_in_camera.header.frame_id = source_frame
    point_in_camera.header.stamp = rospy.Time.now()
    point_in_camera.point.x = point_camera.x
    point_in_camera.point.y = point_camera.y
    point_in_camera.point.z = point_camera.z

    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
        return point_in_base
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("Transformation failed")
        return None

def transform_vector(vector_camera, source_frame, target_frame, tf_buffer):
    vector_in_camera = Vector3Stamped()
    vector_in_camera.header.frame_id = source_frame
    vector_in_camera.header.stamp = rospy.Time.now()
    vector_in_camera.vector.x = vector_camera.x
    vector_in_camera.vector.y = vector_camera.y
    vector_in_camera.vector.z = vector_camera.z

    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        vector_in_base = tf2_geometry_msgs.do_transform_vector3(vector_in_camera, transform)
        return vector_in_base
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("Transformation failed")
        return None
    
def transform_cam2base(surface_camera, approach_camera, binormal_camera, axis_camera ):
    
    # Create a TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Transform the position
    grasp_position_base = transform_point(surface_camera, 'camera_color_optical_frame', 'base_link', tf_buffer)

    grasp_position_base = [grasp_position_base.point.x, grasp_position_base.point.y, grasp_position_base.point.z ]

    # Transform the vectors
    approach_base = transform_vector(approach_camera, 'camera_color_optical_frame', 'base_link', tf_buffer)
    binormal_base = transform_vector(binormal_camera, 'camera_color_optical_frame', 'base_link', tf_buffer)
    axis_base = transform_vector(axis_camera, 'camera_color_optical_frame', 'base_link', tf_buffer)

    approach_base = [approach_base.vector.x, approach_base.vector.y, approach_base.vector.z]
    binormal_base = [binormal_base.vector.x, binormal_base.vector.y, binormal_base.vector.z]
    axis_base = [axis_base.vector.x, axis_base.vector.y, axis_base.vector.z]

    if grasp_position_base is not None and approach_base is not None and binormal_base is not None and axis_base is not None:
        # Normalize the vectors REVISE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        approach_base /= np.linalg.norm(approach_base) # I can delete this section
        binormal_base /= np.linalg.norm(binormal_base) # if the provided vectors
        axis_base /= np.linalg.norm(axis_base)         # are already unit vectors!

        # Create the rotation matrix
        rotation_matrix_base = np.column_stack((approach_base, binormal_base, axis_base))

        # Get the roll, pitch and yaw
        r11, r12, r13 = rotation_matrix_base[0, :]
        r21, r22, r23 = rotation_matrix_base[1, :]
        r31, r32, r33 = rotation_matrix_base[2, :]
        yaw = np.arctan2(r21, r11)
        pitch = np.arcsin(-r31)
        roll = np.arctan2(r32, r33)
    
    return grasp_position_base, rotation_matrix_base, roll, pitch, yaw


def inverse_kinematics(target_position, rotation_matrix):
    my_chain = Chain.from_urdf_file("./urdf/niryo_one.urdf")
    
    ik_result = my_chain.inverse_kinematics(
        target_position = target_position,
        target_orientation = rotation_matrix,
        orientation_mode = "all")
    
    joints = ik_result[1:7]
    return joints

def move_robot_joints(j1, j2, j3, j4, j5, j6):
    rospy.wait_for_service('/niryo_joints_service') #/niryo_joints_service
    try:
        grasp_pose_service = rospy.ServiceProxy('/niryo_joints_service', jointsParm) #/niryo_joints_service
        request = jointsParmRequest(j1, j2, j3, j4, j5, j6)
        response = grasp_pose_service(request)
        if response.success:
            rospy.loginfo("Successfully moved joints %s", response.message)
            return response.success
        else:
            rospy.logwarn("Failed to move joints %s", response.message)
            return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

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

def grasp_object(x, y, z, roll, pitch, yaw):
    rospy.wait_for_service('/niryo_grasp_service')
    try:
        grasp_service = rospy.ServiceProxy('/niryo_grasp_service', graspObject)
        request = graspObjectRequest(x, y, z, roll, pitch, yaw)
        response = grasp_service(request)
        if response.success:
            rospy.loginfo("Successfully grasped object: %s", response.message)
            return response.success
        else:
            rospy.logwarn("Failed to grasp object: %s", response.message)
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
    
if __name__ == "__main__":
    rospy.init_node('robot_control_node')
    
    # Get the grasp configuration for the object
    confirmation = 1
    grasps = get_grasps(confirmation)
    grasp = grasps.grasps[0] # Most possible grasp

    # Transform configuration (pose and orientation) from camera to robot base
    grasp_position_base, rotation_matrix_base, roll, pitch, yaw = transform_cam2base(grasp.surface, grasp.approach, grasp.binormal, grasp.axis)

    # # Inverse kinematics to get joints
    # joints = inverse_kinematics(grasp_position_base, rotation_matrix_base)
    # j1, j2, j3, j4, j5, j6 = joints
    # # Move robot to object
    # confirmation = move_robot_joints(j1, j2, j3, j4, j5, j6)

    x, y, z = grasp_position_base
    rospy.loginfo("x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", x, y, z, roll, pitch, yaw)
    
    # Grasp object
    confirmation = grasp_object(x, y, z, roll, pitch, yaw)

    # # Move to camera 
    # confirmation = move_robot_to_pose(pose)

    # # Identify object and move to classify
    # class_name, confidence = classify_object()
    # class_positions = {'codo':[0, 0, 0], 'neplo':[0, 0, 0], 'tee':[0, 0, 0], 'union':[0, 0, 0]} # Change
    # confirmation = move_robot_to_pose(class_positions[class_name])
    # # STOP GRASPING!!!!!!!!!!!!!!!!!

    # # repeat the above code until there are no more objects


    
    
