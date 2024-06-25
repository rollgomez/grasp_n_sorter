#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from grasp_n_sorter.srv import  jointsParm, jointsParmRequest
from geometry_msgs.msg import PointStamped
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

def transform_cam2base(point_camera):
    
    # Create a TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Define the point in the camera frame
    point_in_camera = PointStamped()
    point_in_camera.header.frame_id = "camera_link"
    point_in_camera.header.stamp = rospy.Time.now()
    point_in_camera.point.x = point_camera.x
    point_in_camera.point.y = point_camera.y
    point_in_camera.point.z = point_camera.z

    # Transform the point to the robot base frame
    try:
        transform = tf_buffer.lookup_transform("base_link", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1.0))
        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
        rospy.loginfo(f"Object coordinates in base frame: {point_in_base.point.x}, {point_in_base.point.y}, {point_in_base.point.z}")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Transformation failed")

    return point_in_base

def inverse_kinematics(points_base):
    my_chain = Chain.from_urdf_file("/home/rolando/catkin_ws/src/niryo_one_description/urdf/niryo_one.urdf")
    point = [points_base.point.x, points_base.point.y, points_base.point.z]
    # point = [0.2, 0.3, 0]
    ik_result = my_chain.inverse_kinematics(point)
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
    rospy.loginfo('Points in camera:')
    rospy.loginfo( grasp.surface)
    points_base = transform_cam2base(grasp.surface)
    rospy.loginfo('Points in base:')
    rospy.loginfo( points_base)
    joints = inverse_kinematics(points_base)
    rospy.loginfo('Move joints to:')
    rospy.loginfo(joints)

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