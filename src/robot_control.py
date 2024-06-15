import rospy
from grasp_n_sorter.srv import reqGrasp, reqGraspResponse, reqGraspRequest
from gpd.msg import GraspConfigList


def get_grasp(confirmation):
    rospy.wait_for_service('select_grasp_service')
    try: 
        get_grasp_client = rospy.ServiceProxy('select_grasp_service', reqGrasp)
        
        response = get_grasp_client(confirmation)
        return response.grasp_configs
    
    except rospy.ServiceException as error:
        rospy.loginfo(error)

if __name__ == '__main__':
    rospy.init_node('robot_control_node')
    confirmation = 1
    grasps = get_grasp(confirmation)
    grasps = grasps.grasps
    grasp = grasps[0]
    # rospy.loginfo(grasp)
    print(grasp.surface)