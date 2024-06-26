#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from grasp_n_sorter.srv import  jointsParm, jointsParmRequest
from geometry_msgs.msg import PointStamped, Vector3Stamped
from grasp_n_sorter.srv import reqGrasp, reqGraspResponse, reqGraspRequest
from gpd.msg import GraspConfigList
from ikpy.chain import Chain

from sensor_msgs.msg import JointState

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
        rospy.logerr("Transformation failed")
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
        rospy.logerr("Transformation failed")
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
        # Normalize the vectors
        approach_base /= np.linalg.norm(approach_base)
        binormal_base /= np.linalg.norm(binormal_base)
        axis_base /= np.linalg.norm(axis_base)

        # Create the rotation matrix
        rotation_matrix_base = np.column_stack((approach_base, binormal_base, axis_base))
    
    return grasp_position_base, rotation_matrix_base


def inverse_kinematics(target_position, rotation_matrix):
    my_chain = Chain.from_urdf_file("/home/rolando/catkin_ws/src/niryo_one_description/urdf/niryo_one.urdf")
    
    ik_result = my_chain.inverse_kinematics(
        target_position = target_position,
        target_orientation = rotation_matrix,
        orientation_mode = "all")
    
    joints = ik_result[1:7]
    return joints

# def request_grasp_pose():

#     rospy.wait_for_service('/niryo_get_joints')
#     try:
#         confirmation = 1
#         grasps = get_grasps(confirmation)
#         grasps = grasps.grasps
#         grasp = grasps[0]
#         # rospy.loginfo(grasp)
#         print(grasp.surface)
#         points_base = transform_cam2base(grasp.surface)
        
#         grasp_pose_service = rospy.ServiceProxy('/niryo_get_joints', jointsParm)
        
#         request = jointsParmRequest(j1, j2, j3, j4, j5, j6)
#         response = grasp_pose_service(request)
#         if response.success:
#             rospy.loginfo("Successfully moved to grasp pose: %s", response.message)
#         else:
#             rospy.logwarn("Failed to move to grasp pose: %s", response.message)
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)

def move():
    confirmation = 1
    grasps = get_grasps(confirmation)
    grasp = grasps.grasps[0]
    grasp_position_base, rotation_matrix_base = transform_cam2base(grasp.surface, grasp.approach, grasp.binormal, grasp.axis)

    rospy.loginfo('Points in camera:')
    rospy.loginfo( grasp.surface)
    rospy.loginfo('Points in base:')
    rospy.loginfo( grasp_position_base)
    rospy.loginfo('Rmatrix camera: ')
    rospy.loginfo(grasp.approach)
    rospy.loginfo(grasp.binormal)
    rospy.loginfo(grasp.axis)
    rospy.loginfo('Rmatrix base: ')
    rospy.loginfo(rotation_matrix_base)

    joints = inverse_kinematics(grasp_position_base, rotation_matrix_base)
    rospy.loginfo('Move joints to:')
    rospy.loginfo(joints)

    # publish joint states to simulate movement
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']  # Add all your joint names
    joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial positions
    joint_state.velocity = []
    joint_state.effort = []

    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        # Update joint_state.position with actual joint values here
        joint_state.position = joints
        joint_pub.publish(joint_state)
        rate.sleep()



if __name__ == "__main__":
    rospy.init_node('robot_control_node')
    try:
        move()
    except rospy.ROSInterruptException:
        pass