## Project: Perception Pick & Place
#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
In the program "project_perception" I wrote a ROS node 'perception' and subscribed to `/pr2/world/points` topic. As the topic contains noisy measurements, I have applied a statistical outlier filter. The outlier filter is based on the assumption of a Gaussian distribution of distances between a given point and its neighbors. Points that are "mean distance + threshold factor * standard deviation" from its k neighbors are considered outliers and removed from the point cloud. After testing different parameters, I chose k to be 50 and threshold factor to be 1.
As an example, please find below a screenshot of the scene before and after applying the filter for scenario 2:

![Cloud Scene](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/cloud_scene.png )

![Cloud Scene Outliers Removed](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/cloud_scene_outliers_filtered.png)

Also, the point cloud changed from a count of 638,283 to 527,571 points. 

To further reduce the number of points in the point cloud without sacrificing important information I have applied a Voxel Grid Downsampling filter. After testing with different leaf size, I chose a value of 0.007. 
For continuity with the previous example, I will demostrate the results using scenario 2. By applying the filter, the number of points was further reduced to 194,269 and the result can be viewed below:

 
![Cloud after Voxel](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/voxel.png)

The next step in the perception pipeline was to isolate a region of interest. This was done by applying a passthrough filter on two axis: z and y. This resulted in a further data reduction (for scenarion 2, from 194,269 to 8291). Please find below the scene after applying the passthrough filters.

 
![Cloud after passthrough](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/passthrough.png)

In order to identify the objects in the pick list, I needed first to discriminate between the objects and the table within the region of interest. To do so, I have applied a RANSAC plane segmentation and retain the outliers (the objects). After experimenting with different "max distance" from the plane model, I was satisfied with the value 0.001 


![Cloud objects only](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/cloud_objects.png)


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

Now that I have isolated the objects in the region of interest, I was interested in segmenting the points in distinct clusters. To accomplish this, I have applied a density based clustering algorithm: Euclidean clustering. After experimenting with different parameters, I chose a cluster tolerance = 0.01, a minimum cluster size of 100 and a maximum cluster size of 2000.
In the following image we can see the clusters identified by the algorithm, where each cluster has a distint color:

 
![Cloud Clustered](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/clustering.png)


#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Once the objects were segmented, it was time to perform object recognition. To accomplish this, I first trained a SVM on features vector extracted for objects in each scenario. The features were a combination of hsv and surface normals signatures. The normalized confusion matrices for each scenario can be seen below:

First Scenario:

![Confusion Matrices 1](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/One/SVM_Confusion_Matrix_1.PNG)

Second Scenario:


![Confusion Matrices 2](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/SVM_Confusion_Matrix_2.PNG)

Third Scenario:

![Confusion Matrices 3](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Three/SVM_Confusion_Matrix_3.PNG)

The model was then used to predict the label for objects in each scenario and a label was output in RViz as it can be seen below:

First Scenario:

![Object Recognition 1](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/One/Object_Recognition_1.PNG)

Second Scenario:

![Object Recognition 2](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Two/Object_Recognition_2.PNG)

Third Scenario:

![Object Recognition 3](https://github.com/rimoune/RoboND-Perception-Project/blob/master/output/Three/Object_Recognition_3.PNG)

It can be seen that the program correctly recognizes 4 out of 5 objects in scenario 2 (it misses "book", which was not included in the scenario) and 6 out of 8 objects in scenario 3 (it misses glue and sticky notes). 

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Once the object recognition step was completed, I could loop through the pick list (relative to a particular tabletop scenario) and write a message to a ".yaml" file with the following information: arm name responsible for picking up the object, the object name, the location of the object on the table, where to drop it and information regarding which scenario we are looking at. From the "/object_list" parameter I've retrieved the object name and the group it belongs to, and from the "/dropbox" parameter the arm name and the drop position. For objects in the pick list that were recognized, the coordinates of the centroid were calculated and served as "pick pose". 
Please follow the link below for the ".yaml" files created: 
https://github.com/rimoune/RoboND-Perception-Project/tree/master/pr2_robot/config/Submission_Jan_11_19 


I have put all the code I have used for this project in pr2_robots/scripts: project_perception.py, capture_features.py, features.py and train_SVM.py. These last 3 scripts were copied over from the sensor_stick folder under the workspace. 
In this project I have completed the requirement of outputing messages with the pick and place positions, however much more can be done. 
Something I would like to explore more is around the features extractions, ways to enrich the input data for each object in a given scenario. Also, I would like to play with different object recognition algorithms and hopefully increase the number of objects recognized in the scene. 
