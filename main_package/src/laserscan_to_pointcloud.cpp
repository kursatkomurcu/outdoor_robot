#include <ros/ros.h>

#include <cmath>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <laser_geometry/laser_geometry.h>


laser_geometry::LaserProjection projector_;
tf::TransformListener* listener_ = nullptr;

ros::Publisher pointCloudPub;
ros::Subscriber laserSub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if(!listener_->waitForTransform(
		scan_in->header.frame_id,
		"base_footprint",
		scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
		ros::Duration(1.0))){
		return;
	}

	sensor_msgs::PointCloud cloud;
	sensor_msgs::PointCloud2 cloud2;
	projector_.transformLaserScanToPointCloud("base_footprint",*scan_in,
	  						cloud, *listener_);
	  
	  
	sensor_msgs::PointCloud cloud_filtered;
	cloud_filtered.header.stamp = ros::Time::now();
	cloud_filtered.header.frame_id = "base_footprint";
	
	geometry_msgs::Point32 point_1; 
	bool first = true;			
	
	const float dzThreshold = 0.05;
				
	for (int i = 0; i < cloud.points.size(); i++) {
		geometry_msgs::Point32 point = cloud.points[i];
		
		if (first == false) {
			float dz = fabs(point.z - point_1.z);
			
			if (dz > dzThreshold) {
				cloud_filtered.points.push_back(point);
			}
		}
		
		first = false;
		point_1 = point;
	}

	sensor_msgs::convertPointCloudToPointCloud2(cloud_filtered, cloud2);
	pointCloudPub.publish(cloud2);

	return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserscan_to_pointcloud");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());
    
    listener_ = new tf::TransformListener;

    ros::Rate rate(5);

    std::string pointCloudTopic, laserScanTopic;

    nh.param<std::string>("pointCloudTopic", pointCloudTopic, "/laser_cloud");
    nh.param<std::string>("laserScanTopic", laserScanTopic, "/laser/scan_filtered");

    laserSub = nh.subscribe<sensor_msgs::LaserScan>(laserScanTopic, 1, scanCallback);
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 1);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    
    ros::waitForShutdown();
    
    delete listener_;

    return 0;
}
