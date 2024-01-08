## Usage of Move Base Node with Surface Normal Vector Slopes:
    $ roslaunch main_package run_gazebo_world.launch world_file:=clearpath_playpen.world
    $ roslaunch main_package run_simulation.launch mapping_file:=sensor_fusion.launch
    $ roslaunch main_package code.launch thresholdDegree:=10
    $ roslaunch main_package move_base.launch 
    
    
In this tutorial, we try to autonomous driving based on surface slopes that sensing and calculating from depth camera. First of all, we subscribe **/rtabmap/cloud_map** topic to find surface normal vector. After that, we used **KdTree algorithm** and found the vectors. This algorithm organizes the data points in a binary tree structure, dividing them along different axes. This allows for quick nearest neighbor searches. The algorithm works by recursively partitioning the data and selecting the splitting planes based on the median values. During searching, it traverses the tree, comparing the search point to the splitting planes to determine the appropriate child node. It continues until a leaf node is reached, and then backtracks to find the closest data point. KdTree is useful for applications that require fast nearest neighbor queries in multi-dimensional space. https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html

![](https://github.com/kursatkomurcu/Machine-Learning/blob/main/kdtree.png)

There may be noises in point cloud data so we used **Statistical Outlier Removal** method for remove the noise. This method calculates mean and standard deviation in the data. Each data point is checked to see if it falls outside the expected range defined by the threshold. If a data point is significantly different from the average or deviates too much from the rest of the data, it is flagged as an outlier. Then it deletes this datas. https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html

After removing the noise, we calculate slopes the code in below: 

      
      float slope = std::sqrt(point.normal_x * point.normal_x + point.normal_y * point.normal_y);
      float slopeDegrees = (std::atan(point.normal_z / slope) * 180.0 / M_PI);
      float slopePercentage = (slopeDegrees / REFERENCE_ANGLE) * 100.0;
      
      
Then we publish as occupancy grid and marker array.

![](https://github.com/kursatkomurcu/Machine-Learning/blob/main/move_base_test.jpeg)

# Launch gazebo world:
    
    $ roslaunch main_package run_gazebo_world.launch world_file:=iq_map_deneme.world x:=-30 y:=0 z:=5
        
# Launch the simulation file:

    $ roslaunch main_package run_simulation.launch mapping_file:=sensor_fusion.launch
    
# Estimate the Surface Normal Vectors and Get GPS Data
You can write thresholdDegree as parameter in terminal. The degree should be between 0-90. Default is 10 degree

    $ roslaunch main_package code.launch thresholdDegree:=10
    
# Launch move base file:

    $ roslaunch main_package move_base.launch 
   
After launch all nodes, you can click 2D Nav Goal button in RViz and mark any point you want. The robot will go to the point autonomously. Or if you want to write to terminal of your longitute latitude and altitude values, use the following command.

    $ rostopic pub --once /move_base_simple/goal_fix sensor_msgs/NavSatFix "header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    status: {status: 0, service: 0}
    latitude: 49.89976957559654
    longitude: 8.90000548264246
    altitude: -1.0112892212547795
    position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"


