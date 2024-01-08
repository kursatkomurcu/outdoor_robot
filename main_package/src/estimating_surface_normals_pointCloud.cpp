#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#define REFERENCE_ANGLE 45.0

static const std::string IMAGE_TOPIC = "/octomap_point_cloud_centers";
static const std::string TF_TOPIC = "/tf";
static const std::string ODOM_TOPIC = "/odometry/filtered_fusion";

static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string OCCUPANCY_GRID_TOPIC = "/occupancy_grid";

std::string pointCloudTopic = "";
std::string occupancyGridTopic = "";

nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped tf_msg;

ros::Publisher pub;
ros::Publisher occupancyGridPub;

ros::Subscriber sub;
ros::Subscriber tfSub;
ros::Subscriber odomSub;

float thresholdDegree;
float distance;

struct PositionData {
    double x;
    double y;
    double z;
};

PositionData position;
bool positionReceived = false;



void publishOccupancyGrid(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, float thresholdDegree)
{
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.header = pcl_conversions::fromPCL(cloud_with_normals->header);
    occupancyGrid.header.frame_id = "map";
    occupancyGrid.header.stamp = ros::Time::now();
        
    // Set the resolution of the occupancy grid (in meters per cell)
    double resolution = 0.2; // Adjust this value according to your needs
    occupancyGrid.info.resolution = resolution;

    // Calculate the grid dimensions based on the point cloud extents
    pcl::PointXYZRGBNormal minPt, maxPt;
    
    if(cloud_with_normals->empty()){
	    minPt.x = minPt.y = minPt.z = -5.0;
	    maxPt.x = maxPt.y = maxPt.z = 5.0;
    }    
    else {
	    minPt.x = minPt.y = minPt.z = std::numeric_limits<float>::max();
	    maxPt.x = maxPt.y = maxPt.z = -std::numeric_limits<float>::max();

	    for (const auto& point : *cloud_with_normals) {
		if (point.x < minPt.x)
		    minPt.x = point.x;
		if (point.y < minPt.y)
		    minPt.y = point.y;
		if (point.z < minPt.z)
		    minPt.z = point.z;

		if (point.x > maxPt.x)
		    maxPt.x = point.x;
		if (point.y > maxPt.y)
		    maxPt.y = point.y;
		if (point.z > maxPt.z)
		    maxPt.z = point.z;
	    }
    }
    
    const double tol = 10.0;

    double gridWidth = std::ceil((maxPt.x - minPt.x + 2 * tol) / resolution + 1);
    double gridHeight = std::ceil((maxPt.y - minPt.y + 2 * tol) / resolution + 1);
    occupancyGrid.info.width = gridWidth;
    occupancyGrid.info.height = gridHeight;

    // Calculate the origin of the occupancy grid
    occupancyGrid.info.origin.position.x = minPt.x - tol;
    occupancyGrid.info.origin.position.y = minPt.y - tol;
    
    occupancyGrid.info.origin.orientation.w = 1.0;

    // Initialize the grid data as unknown (-1)
    occupancyGrid.data.assign(gridWidth * gridHeight, 0);

    // Pre-calculate the inverse resolution
    double inv_resolution = 1.0 / resolution;

    // Iterate over the point cloud and update the occupancy grid
    for (const auto& point : *cloud_with_normals) {
        // Calculate the slope percentage for the current point
        float slope = atan2(point.normal_z, sqrt(point.normal_x * point.normal_x + point.normal_y * point.normal_y)) * 180.0 / M_PI;

        // Calculate the grid indices for the current point
        int gridX = std::floor((point.x - occupancyGrid.info.origin.position.x) * inv_resolution);
        int gridY = std::floor((point.y - occupancyGrid.info.origin.position.y) * inv_resolution);
        int gridIndex = gridY * occupancyGrid.info.width + gridX;

        // Update the occupancy grid cell based on the slope percentage and threshold
        int occupancy = -1; // Default value for unknown cells

        // if (slope > 90)
        //    slope = fabs(180 - slope);

	
	double slopeDec = slope;
	if (slope < 0) {
		slopeDec = fabs(-90.0 - slope);
	}
	else {
		slopeDec = fabs(90.0 - slope);
	}
	
        double dist = sqrt((point.x - position.x)*(point.x - position.x) + 
        			(point.y - position.y)*(point.y - position.y) + 
        			(point.z - position.z)*(point.z - position.z) );
        if (slopeDec >= thresholdDegree && 
        	dist >= 1.0 /* meters */) {
            occupancy = 100; // Occupied cell
            
            /*
            std::cout << "Slope Degree: " << slopeDec << std::endl;
		
            std::cout << "X: " << point.x << std::endl;
            std::cout << "Y: " << point.y << std::endl;
            std::cout << "Z: " << point.z << std::endl;

            std::cout << "Normal X: " << point.normal_x << std::endl;
            std::cout << "Normal Y: " << point.normal_y << std::endl;
            std::cout << "Normal Z: " << point.normal_z << std::endl;
	    */
            
        } else {
            occupancy = 0; // Free cell
        }
        occupancyGrid.data[gridY * occupancyGrid.info.width + gridX] = occupancy;
    }
    

    // Publish the occupancy grid
    occupancyGridPub.publish(occupancyGrid);
}

void tfCallBack(const tf2_msgs::TFMessage::ConstPtr& msg){
    for (const auto& transform : msg->transforms) {
        if (transform.header.frame_id == "map" && transform.child_frame_id == "base_link"){
            tf_msg = transform;
            
            position.x = tf_msg.transform.translation.x;
	        position.y = tf_msg.transform.translation.y;
	        position.z = tf_msg.transform.translation.z;
	    
            break;
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_msg = *msg;
    
    position.x = odom_msg.pose.pose.position.x;
    position.y = odom_msg.pose.pose.position.y;
    position.z = odom_msg.pose.pose.position.z;
    
    positionReceived = true;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pt(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

   float minX = position.x - distance;
   float minY = position.y - distance;
   float minZ = position.z - distance;

   float maxX = position.x + distance;
   float maxY = position.y + distance;
   float maxZ = position.z + distance;
   
   cloud_pt->clear();

   for (const auto point: *cloud) {
      if (pcl::isFinite(point) == false || 
      		std::isnan(point.x) == true || std::isnan(point.y) == true || std::isnan(point.z) == true) {
      		continue;
      }
      else {
	      //ROS_WARN("Point: %f %f %f", point.x, point.y, point.z);
      }

      if (	point.x > maxX || point.x < minX || 
	   	point.y > maxY || point.y < minY ||
	   	point.z > maxZ || point.z < minZ) {
			
      }
      else {
      		cloud_pt->push_back(point);
      }
   }

    
    if (positionReceived == false) {
    	return;
    }

    ROS_INFO("position = %f, %f, %f", position.x, position.y, position.z);

    if(cloud_pt->empty()){
        //cloud_pt = cloud;
        ROS_WARN("PassThrough result is empty. Used point cloud without PassThrough Filter");
    }
        
    //voxel grid filter
    /*pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud_pt);
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_grid.filter(*cloud_pt);*/

    // Estimate surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    ne.setInputCloud(cloud_pt);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod(tree);
    ne.setKSearch(25);  // Number of nearest neighbors to use for estimating surface normals
    //ne.setRadiusSearch (0.025);
    ne.compute(*normals);

    // Create a new point cloud with XYZRGBNormal data type
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    cloud_with_normals->reserve(cloud_pt->size());
    pcl::concatenateFields(*cloud_pt, *normals, *cloud_with_normals);

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud(cloud_with_normals);
    sor.setMeanK(50);         // The number of points around a reference to determine whether a point is out of group
    sor.setStddevMulThresh(0.05);   // Standard deviation threshold
    sor.filter(*cloud_with_normals_filtered);

    publishOccupancyGrid(cloud_with_normals_filtered, thresholdDegree);

    // Convert the pcl::PointCloud<pcl::PointXYZRGBNormal> to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_with_normals_filtered, output);

    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roscpp_pcl_example");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

    ros::Rate rate(5);
    nh.param<float>("thresholdDegree", thresholdDegree, 50.0);
    nh.param<float>("distance", distance, 20.0);

    nh.param<std::string>("pointCloudTopic", pointCloudTopic, std::string(IMAGE_TOPIC));
    nh.param<std::string>("occupancyGridTopic", occupancyGridTopic, std::string(PUBLISH_TOPIC));

    odomSub = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 1, odomCallback);
    //tfSub = nh.subscribe<tf2_msgs::TFMessage>(TF_TOPIC, 1, tfCallBack);
    sub = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    occupancyGridPub = nh.advertise<nav_msgs::OccupancyGrid>(occupancyGridTopic, 1);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    
    ros::waitForShutdown();

    return 0;
}
