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
#include <pcl/kdtree/kdtree.h>

#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

static const std::string IMAGE_TOPIC = "/laser_cloud";
static const std::string TF_TOPIC = "/tf";
static const std::string ODOM_TOPIC = "/odometry/filtered_fusion";

static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string LASER_MOVE_BASE_TOPIC = "/laser_cloud_move_base";

std::string pointCloudTopic = "";
std::string occupancyGridTopic = "";

nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped tf_msg;

ros::Publisher pub;
ros::Publisher derivativeCloudPub;

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

void publishOccupancyGrid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_normals, float thresholdDegree)
{       
    // Set the resolution of the occupancy grid (in meters per cell)
    double resolution = 0.2; // Adjust this value according to your needs

    // Pre-calculate the inverse resolution
    double inv_resolution = 1.0 / resolution;
    
    // Initialize a KD-Tree for nearest neighbor search
    pcl::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_with_normals);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr derivative_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    derivative_cloud->reserve(cloud_with_normals->size());

    // Iterate over the point cloud and update the occupancy grid
    for (const auto& point : *cloud_with_normals) {
        double dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

        std::vector<int> point_indices;
        std::vector<float> point_distances;

        pcl::PointXYZRGB search_point;
        search_point.x = point.x;
        search_point.y = point.y;
        search_point.z = point.z;

        int k = 5;
        kdtree->nearestKSearch(search_point, k, point_indices, point_distances);

        double z_derivative = 0.0;
        double dzThresh = 0.1; // meters
        if (!point_indices.empty()){
            double z_value = point.z;
            double z_value_neighbor = 0.0;

            for (size_t i = 0; i < point_indices.size(); ++i){
                z_value_neighbor = cloud_with_normals->points[point_indices[i]].z;

                // std::cout << "Z Value: " << z_value << std::endl;
                // std::cout << "Z Value Neigbors: " << z_value_neighbor << std::endl;
                
                if (fabs(z_value - z_value_neighbor) > dzThresh && dist > 1.0) {
                    std::cout << "Z Value IF: " << z_value << std::endl;
                    std::cout << "Z Value Neigbors IF: " << z_value_neighbor << std::endl;
                    derivative_cloud->push_back(point);
                    break;
                }
            }

        }
    }
    
    sensor_msgs::PointCloud2 derivative_cloud_msg;
    pcl::toROSMsg(*derivative_cloud, derivative_cloud_msg);

    derivative_cloud_msg.header.frame_id = "base_footprint";
    derivative_cloud_msg.header.stamp = ros::Time::now();

    // Publish the occupancy grid
    derivativeCloudPub.publish(derivative_cloud_msg);
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

    publishOccupancyGrid(cloud_pt, thresholdDegree);

    // Convert the pcl::PointCloud<pcl::PointXYZRGBNormal> to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_pt, output);

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

    odomSub = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 1, odomCallback);
    //tfSub = nh.subscribe<tf2_msgs::TFMessage>(TF_TOPIC, 1, tfCallBack);
    sub = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    derivativeCloudPub = nh.advertise<sensor_msgs::PointCloud2>(LASER_MOVE_BASE_TOPIC, 1);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    
    ros::waitForShutdown();

    return 0;
}
