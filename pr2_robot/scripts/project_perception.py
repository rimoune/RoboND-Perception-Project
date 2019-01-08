#!/usr/bin/env python
#RL 7Jan Change statistical filter:     outlier_filter.set_mean_k(30) -->     outlier_filter.set_mean_k(50)
#RL 7Jan Change statistical filter:     x = 2.0 -->     x = 0.05
#RL 7Jan x = 0.05 --> 1.0

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
from rospy_message_converter import message_converter
import yaml
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

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud=ros_to_pcl(pcl_msg)
    pcl.save(cloud,"./rima_output/cloud_scene.pcd")
    
    # TODO: Statistical Outlier Filtering # RL added on 12/19
    #Outlier Removal Filter
    outlier_filter = cloud.make_statistical_outlier_filter()# RL added on 12/19

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    #pcl.save(cloud_filtered, "./rima_output/cloud_filtered.pcd") #RL added on Jan2 to play with statistical filtering
    #outlier_filter.set_negative(True)#RL added on Jan2 to play with statistical filtering
    #pcl.save(outlier_filter.filter(),"./rima_output/cloud_filtered_outliers.pcd")#RL added on Jan2 to play with statistical filtering

    # TODO: Voxel Grid Downsampling
    
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter() #RL added on 12/19
    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.007   
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()


    # TODO: PassThrough Filter
    
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    #0.5-2 seemed good to me 
    axis_min = 0.6
    axis_max = 2.0
    passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()

    #RL added the following pass through on 28/12
    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    #0.5-2 seemed good to me 
    axis_min = -0.3
    axis_max = 0.3
    passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    #end 28/12


    #filename = 'pass_through_filtered.pcd' #RL added on 28/12 
    #pcl.save(cloud_filtered, filename) #RL added on 28/12

    # TODO: RANSAC Plane Segmentation

    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
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
 

    # TODO: Euclidean Clustering
    
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)#changed on Jan3 from 10 to 100
    ec.set_MaxClusterSize(2000)
	# Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()


    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []
 
    for j, indices in enumerate(cluster_indices):
        
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2],rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_table =pcl_to_ros(cloud_table)
    ros_cloud_objects =pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4 #RL changed from 0.4 to 0.25 back to 0.4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
        

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        #pr2_mover(detected_objects_list) commented out RL 12/15/2018 error detected_objects_list not defined
        pr2_mover(detected_objects)       # added RL 12/15/2018 error detected_objects_list not defined
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    test_scene_num = Int32() # RL added on 12/17/18
    test_scene_num.data = 3 # RL How To retrieve this automatically?
    
    object_name = String()  # RL added on 12/17/18
    arm_name = String() # ci torno 
    pick_pose=Pose() # RL added on 12/17/18
    place_pose=Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    #print 'Object List Param: ',object_list_param
    #print 'Type Object List Param: ',type(object_list_param)
    place_param = rospy.get_param('/dropbox')    



    # TODO: Rotate PR2 in place to capture side tables for the collision map ?

    # TODO: Loop through the pick list
    dict_list = [] # RL added on 12/17/18

    for i in range (len(object_list_param)):

        # TODO: Parse parameters into individual variables
        object_pick_name = object_list_param[i]['name']
        print "I: ",i, "object name: ", object_pick_name
        object_name.data = object_list_param[i]['name']#RL added 12/17/18

        object_pick_group = object_list_param[i]['group']
        print "I: ",i, "object group: ", object_pick_group

	if object_pick_group == "red":
            for item in place_param:
                if item['group'] == "red":
                    # TODO: Assign the arm to be used for pick_place
                    arm_name.data = item['name']
                    # TODO: Create 'place_pose' for the object
                    place_pose.position.x = item['position'][0]
                    place_pose.position.y = item['position'][1]
                    place_pose.position.z = item['position'][2]
        if object_pick_group == "green":
            for item in place_param:
                if item['group'] == "green":
                    # TODO: Assign the arm to be used for pick_place
                    arm_name.data = item['name']
                    # TODO: Create 'place_pose' for the object
                    place_pose.position.x = item['position'][0]
                    place_pose.position.y = item['position'][1]
                    place_pose.position.z = item['position'][2]
  



        # TODO: Get the PointCloud for a given object and obtain it's centroid
        labels=[]
        centroids =[]
        
        for object in object_list:
             if (object_pick_name == object.label):
                  labels.append(object.label)         
                  points_arr = ros_to_pcl(object.cloud).to_array()
                  centroids.append(np.mean(points_arr, axis=0)[:3])   
        	  print "Labels: ", labels  
                  print "Centroids: ", centroids 
        	  pick_pose.position.x = float(centroids[0][0]) # RL added on 12/17/18
	          pick_pose.position.y = float(centroids[0][1]) # RL added on 12/17/18
                  pick_pose.position.z = float(centroids[0][2])# RL added on 12/17/18

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose) #RL added on 12/17/18
        
        dict_list.append(yaml_dict)
        print dict_list
 
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE) RL commented out on 12/17/18 because of the which_arm
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    # TODO: Output your request parameters into output yaml file # RL added on 12/17/18
    yaml_filename ='/home/robond/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/output_3.yaml' 
    send_to_yaml(yaml_filename, dict_list)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub=rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    
    object_markers_pub=rospy.Publisher("/object_markers",Marker,queue_size=1)
    detected_objects_pub =rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
     
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
