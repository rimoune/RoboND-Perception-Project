#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
#from rospy_message_converter import message_converter
#import yaml
from std_msgs.msg import Int32 #RL added on 12/17/18 non serve
from std_msgs.msg import String #RL added on 12/17/18 non serve





# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)
 

p = pcl.load_XYZRGB("./rima_output/cloud_scene.pcd")
pcl.save(p,"./rima_output/cloud_scene2.pcd")
    
    # TODO: Statistical Outlier Filtering # RL added on 12/19
    #Outlier Removal Filter
outlier_filter = p.make_statistical_outlier_filter()# RL added on 12/19

    # Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)

    # Set threshold scale factor
x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()

pcl.save(cloud_filtered, "./rima_output/cloud_filtered.pcd") #RL added on Jan2 to play with statistical filtering
    #outlier_filter.set_negative(True)#RL added on Jan2 to play with statistical filtering
    #pcl.save(outlier_filter.filter(),"./rima_output/cloud_filtered_outliers.pcd")#RL added on Jan2 to play with statistical filtering

    # TODO: Voxel Grid Downsampling
    
    # Create a VoxelGrid filter object for our input point cloud
vox = cloud_filtered.make_voxel_grid_filter() #RL added on 12/19
    # Choose a voxel (also known as leaf) size
LEAF_SIZE = 0.007 #0.001 too small warning   
    # Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = './rima_output/voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)


    # TODO: PassThrough Filter
    
    # Create a PassThrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
    #0.6-2 seemed good to me 
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough.filter()

    #RL added the following pass through on 28/12
passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
    #0.5-2 seemed good to me 
axis_min = -0.3 # da 0.5 a 0.3 on Jan3
axis_max = 0.3  # da 0.5 a 0.3 on Jan3
passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough.filter()
    #end 28/12


filename = './rima_output/pass_through_filtered.pcd' #RL added on 28/12 
pcl.save(cloud_filtered, filename) #RL added on 28/12

    # TODO: RANSAC Plane Segmentation

    # Create the segmentation object
seg = cloud_filtered.make_segmenter()
    # Modelling the table as a plane
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
max_distance = 0.001
seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()


    # TODO: Extract inliers and outliers
    
    # Extract inliers (Table)
cloud_table = cloud_filtered.extract(inliers, negative=False)
    # Extract outliers (Objects)
cloud_objects = cloud_filtered.extract(inliers, negative=True)
filename = "./rima_output/cloud_table.pcd"
pcl.save(cloud_table, filename)
filename = "./rima_output/cloud_objects.pcd"
pcl.save(cloud_objects, filename) 

    # TODO: Euclidean Clustering (DBSCAN)
    
white_cloud = XYZRGB_to_XYZ(cloud_objects)
tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	
#ec.set_ClusterTolerance(0.025)
ec.set_ClusterTolerance(0.01)

ec.set_MinClusterSize(100)# changed from 10 to 100 on Jan3
ec.set_MaxClusterSize(2000)
	# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()


    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
get_color_list.color_list = [] #RL added on Jan2 to test (moved from main)
cluster_color = get_color_list(len(cluster_indices))
print "cluster_color**", len(cluster_indices)
color_cluster_point_list = []
 
for j, indices in enumerate(cluster_indices):       
    for i, indice in enumerate(indices):
        color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2],rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
filename = "./rima_output/clustering.pcd"
pcl.save(cluster_cloud, filename)



