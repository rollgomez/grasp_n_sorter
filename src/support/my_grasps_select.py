import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
from scipy.linalg import lstsq
from gpd.msg import CloudIndexed
from std_msgs.msg import Header, Int64
from geometry_msgs.msg import Point
from gpd.msg import GraspConfigList
import sensor_msgs.point_cloud2 as pc2
import pcl

# Global variables
cloud = []
grasps = []

def filter_pointcloud(pointcloud):
    # Define the workspace limits
    x_min, x_max = -0.16, 0.07
    y_min, y_max = -0.5, 0.5
    z_min, z_max = 0.51, 0.63

    # Extract points within the specified workspace
    filtered_points = []
    for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point[0], point[1], point[2]
        if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
            filtered_points.append(point)

    # Convert filtered points to a numpy array
    return np.array(filtered_points)

# Callback function to process the point cloud data
def cloudCallback(msg):
    global cloud
    if len(cloud) == 0:
        cloud = filter_pointcloud(msg)

# Create a ROS node
rospy.init_node('select_grasp')

# Subscribe to the ROS topic that contains the point cloud
cloud_sub = rospy.Subscriber('/stereo_publisher/stereo/points', PointCloud2, cloudCallback)

# Publisher for the filtered point cloud
filtered_pub = rospy.Publisher('/filtered_cloud', PointCloud2, queue_size=1)

# Wait for the point cloud to arrive
while len(cloud) == 0:
    rospy.sleep(0.005)

# Convert the cloud to a numpy array and remove any rows with NaN or inf values
cloud = np.array(cloud)
cloud = cloud[np.isfinite(cloud).all(axis=1)]

# Publish the filtered point cloud
header = Header()
header.frame_id = "base_link"
header.stamp = rospy.Time.now()
filtered_cloud_msg = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
filtered_pub.publish(filtered_cloud_msg)

# Extract the nonplanar indices
X = cloud
A = np.c_[X[:,0], X[:,1], np.ones(X.shape[0])]
C, _, _, _ = lstsq(A, X[:,2])
a, b, c, d = C[0], C[1], -1., C[2] # coefficients of the form: a*x + b*y + c*z + d = 0.
dist = ((a*X[:,0] + b*X[:,1] + d) - X[:,2])**2
err = dist.sum()
idx = np.where(dist > 0.005)

# Publish point cloud and nonplanar indices
pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1)

msg = CloudIndexed()
header = Header()
header.frame_id = "base_link"
header.stamp = rospy.Time.now()
msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud.tolist())
msg.cloud_sources.view_points.append(Point(0,0,0))
for i in range(cloud.shape[0]):
    msg.cloud_sources.camera_source.append(Int64(0))
for i in idx[0]:
    msg.indices.append(Int64(i))    
s = input('Hit [ENTER] to publish')
pub.publish(msg)
rospy.sleep(2)
print('Published cloud with', len(msg.indices), 'indices')

# Callback function to process the grasps data
def graspsCallback(msg):
    global grasps
    grasps = msg.grasps

# Subscribe to the ROS topic that contains the grasps
grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, graspsCallback)

# Wait for grasps to arrive
rate = rospy.Rate(1)
while not rospy.is_shutdown():    
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break

grasp = grasps[0] # grasps are sorted in descending order by score
print('Selected grasp with score:', grasp.score)
