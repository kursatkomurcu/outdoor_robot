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
#include <pcl/segmentation/extract_clusters.h>

#include <cmath>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

static const std::string IMAGE_TOPIC = "/octomap_point_cloud_centers";
static const std::string TF_TOPIC = "/tf";

static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string NEGATIVE_VOLUME_TOPIC = "/occupancy_grid_negative_volume";

geometry_msgs::TransformStamped tf_msg;

ros::Publisher pub;
ros::Publisher negativeVolumePub;

ros::Subscriber sub;
ros::Subscriber tfSub;

struct PositionData {
    double x;
    double y;
    double z;
};

PositionData position;

const int kernelSize = 3;
const int kernelMid = kernelSize / 2;
int kernel[kernelSize][kernelSize] = {
    {1, 1, 1},
    {1, 1, 1},
    {1, 1, 1}
};

nav_msgs::OccupancyGrid erosion(nav_msgs::OccupancyGrid occupancyGrid){
    nav_msgs::OccupancyGrid morphologyGrid = occupancyGrid;
    for(int y = 0; y < occupancyGrid.info.height; ++y){
        for(int x = 0; x < occupancyGrid.info.width; ++x){
           int currentIndex = y * occupancyGrid.info.width + x;

            if(occupancyGrid.data[currentIndex] == 0){
                bool shouldErode = true;

                for(int ky = 0; ky < kernelSize; ++ky){
                    for(int kx = 0; kx < kernelSize; ++kx){
                        int neighborX = x + kx - kernelMid;
                        int neighborY = y + ky - kernelMid;

                        if (neighborX >= 0 && neighborX < occupancyGrid.info.width &&
                            neighborY >= 0 && neighborY < occupancyGrid.info.height) {

                            int neighborIndex = neighborY * occupancyGrid.info.width + neighborX;

                            // Check if the neighboring cell is occupied (100) but the kernel is not matching
                            if (occupancyGrid.data[neighborIndex] == 100 && kernel[ky][kx] != 1) {
                                shouldErode = false;
                                break;
                            }
                        }
                    }

                    if (!shouldErode) {
                            break;
                    }
                } 

                // Perform erosion by setting the current cell to unoccupied (0) if all kernel elements match
                if (shouldErode) {
                    morphologyGrid.data[currentIndex] = 0;
                }
            }
        }
    }
    return morphologyGrid;
}

nav_msgs::OccupancyGrid dilation(nav_msgs::OccupancyGrid occupancyGrid){
    nav_msgs::OccupancyGrid morphologyGrid = occupancyGrid;
    for(int y = 0; y < occupancyGrid.info.height; ++y){
        for(int x = 0; x < occupancyGrid.info.width; ++x){
            int currentIndex = y * occupancyGrid.info.width + x;

            if(occupancyGrid.data[currentIndex] == 100){
                for(int ky = 0; ky < kernelSize; ++ky){
                    for(int kx = 0; kx < kernelSize; ++kx){
                        int neighborX = x + kx - kernelMid;
                        int neighborY = y + ky - kernelMid;

                        if (neighborX >= 0 && neighborX < occupancyGrid.info.width &&
                            neighborY >= 0 && neighborY < occupancyGrid.info.height){
                            
                            int neighborIndex = neighborY * occupancyGrid.info.width + neighborX;

                            // Perform dilation by setting the neighboring cell to occupied (100) if the kernel is 1
                            if (kernel[ky][kx] == 1) {
                                morphologyGrid.data[neighborIndex] = 100;
                            }
                        }
                    }
                }
            }
        }
    }
    return morphologyGrid;
}

int calculateContourArea(nav_msgs::OccupancyGrid& occupancyGrid, int x, int y, int width, int height, int contourValue){
    int area = 0;
    std::vector<std::pair<int, int>> stack;
    stack.emplace_back(x, y);

    while(!stack.empty()){
        int cx, cy;
        std::tie(cx, cy) = stack.back();
        stack.pop_back();

        if(cx < 0 || cx >=width || cy < 0 || cy >=height)
            continue;
        
        int index = cy * width + cx;
        if(occupancyGrid.data[index] == contourValue){
            occupancyGrid.data[index] = -1;
            area++;

            stack.emplace_back(cx - 1, cy);
            stack.emplace_back(cx + 1, cy);
            stack.emplace_back(cx, cy - 1);
            stack.emplace_back(cx, cy + 1);
        }
    }
    return area;
}

nav_msgs::OccupancyGrid findAndFillContours(nav_msgs::OccupancyGrid occupancyGrid, int minAreaThreshold) {
    int width = occupancyGrid.info.width;
    int height = occupancyGrid.info.height;
    int contourValue = 100;
    int index;

    // Create a copy of the input occupancyGrid to avoid modifying the original one
    nav_msgs::OccupancyGrid filledGrid = occupancyGrid;

    // Find contours by checking neighboring points and mark them with contourValue
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            index = y * width + x;

            if (filledGrid.data[index] == 0) {
                if (filledGrid.data[y * width + (x - 1)] == contourValue ||
                    filledGrid.data[y * width + (x + 1)] == contourValue ||
                    filledGrid.data[(y - 1) * width + x] == contourValue ||
                    filledGrid.data[(y + 1) * width + x] == contourValue) {
                    filledGrid.data[index] = -1;
                }
            }
        }
    }

    // Now, fill the interior of the contours with black (100)
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            index = y * width + x;
            if (filledGrid.data[index] == -1) {
                // Check if any neighboring cell is not part of a contour
                if (filledGrid.data[y * width + (x - 1)] == contourValue ||
                    filledGrid.data[y * width + (x + 1)] == contourValue ||
                    filledGrid.data[(y - 1) * width + x] == contourValue ||
                    filledGrid.data[(y + 1) * width + x] == contourValue) {
                    // This cell is part of the interior, fill it with black (100)
                    filledGrid.data[index] = 100;
                }
            }
        }
    }

    // Calculating contour area and remove them which is lower than threshold
    for(int y = 0; y < height; ++y){
        for(int x = 0; x < width; ++x){
            index = y * width + x;
            if(filledGrid.data[index] == contourValue){
                int area = calculateContourArea(filledGrid, x, y, width, height, contourValue);
                if(area < minAreaThreshold){
                    for(int yy = 0; yy < height; ++yy){
                        for(int xx = 0; xx < width; ++xx){
                            if(filledGrid.data[yy * width + xx] == -1){
                                filledGrid.data[yy * width + xx] = 0;
                            }
                        }
                    }
                } else {
                    // The contour is valid, fill its interior with 100
                    std::vector<std::pair<int, int>> stack;
                    stack.emplace_back(x, y);

                    while (!stack.empty()) {
                        int cx, cy;
                        std::tie(cx, cy) = stack.back();
                        stack.pop_back();

                        if (cx < 0 || cx >= width || cy < 0 || cy >= height) continue;

                        int index = cy * width + cx;
                        if (filledGrid.data[index] == -1) {
                            filledGrid.data[index] = 100;

                            stack.emplace_back(cx - 1, cy);
                            stack.emplace_back(cx + 1, cy);
                            stack.emplace_back(cx, cy - 1);
                            stack.emplace_back(cx, cy + 1);
                        }
                    }
                }
            }
        }
    }

    return filledGrid;
}

void publishNegativeVolume(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals){
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.header = pcl_conversions::fromPCL(cloud_with_normals->header);

    // Set the resolution of the occupancy grid (in meters per cell)
    double resolution = 0.2; // Adjust this value according to your needs
    occupancyGrid.info.resolution = resolution;

    // Check if the point cloud is empty
    if (cloud_with_normals->empty()) {
        // If the point cloud is empty, set the occupancy grid data to all zeros
        occupancyGrid.info.resolution = 0.2; // Adjust this value according to your needs

        // Set the desired size of the empty grid (in meters)
        double emptyGridSize = 50.0;
        double emptyGridWidth = std::ceil(emptyGridSize / resolution + 1);
        double emptyGridHeight = std::ceil(emptyGridSize / resolution + 1);

        occupancyGrid.info.width = emptyGridWidth;
        occupancyGrid.info.height = emptyGridHeight;

        // Calculate the origin of the occupancy grid to center the empty grid
        occupancyGrid.info.origin.position.x = -emptyGridSize / 2.0;
        occupancyGrid.info.origin.position.y = -emptyGridSize / 2.0;

        occupancyGrid.data.assign(emptyGridWidth * emptyGridHeight, 0);

        // Publish the empty grid and return
        negativeVolumePub.publish(occupancyGrid);
        return;
    }

    // Calculate the grid dimensions based on the point cloud extents
    pcl::PointXYZRGBNormal minPt, maxPt;
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

    const double tol = 10.0;

    double gridWidth = std::ceil((maxPt.x - minPt.x + 2 * tol) / resolution + 1);
    double gridHeight = std::ceil((maxPt.y - minPt.y + 2 * tol) / resolution + 1);
    occupancyGrid.info.width = gridWidth;
    occupancyGrid.info.height = gridHeight;

    // Calculate the origin of the occupancy grid
    occupancyGrid.info.origin.position.x = minPt.x - tol;
    occupancyGrid.info.origin.position.y = minPt.y - tol;

    occupancyGrid.data.assign(gridWidth * gridHeight, 0);

    // Pre-calculate the inverse resolution
    double inv_resolution = 1.0 / resolution;

    for (const auto& point : *cloud_with_normals){
        // Calculate the grid indices for the current point
        int gridX = std::floor((point.x - occupancyGrid.info.origin.position.x) * inv_resolution);
        int gridY = std::floor((point.y - occupancyGrid.info.origin.position.y) * inv_resolution);
        int gridIndex = gridY * occupancyGrid.info.width + gridX;

        std::cout << "ROBOT Z: " << position.z << std::endl;
        std::cout << "Point Z: " << point.z << std::endl;

        int occupancy = -1;

        if(fabs(point.z) < fabs(position.z)) // 2 is tolerance
            occupancy = 100;
        else
            occupancy = 0;
        
        occupancyGrid.data[gridY * occupancyGrid.info.width + gridX] = occupancy;
    }

    // dilation & erosion
    nav_msgs::OccupancyGrid morphologyGrid = occupancyGrid;

    morphologyGrid = dilation(morphologyGrid);
    morphologyGrid = erosion(morphologyGrid);

    morphologyGrid = findAndFillContours(morphologyGrid, 150); // 10 --> area threshold

    negativeVolumePub.publish(morphologyGrid);
    
}

void tfCallBack(const tf2_msgs::TFMessage::ConstPtr& msg){
    for (const auto& transform : msg->transforms) {
        if (transform.header.frame_id == "map"){
            tf_msg = transform;
            break;
        }
    }
}

void callBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // Convert the sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pt(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    position.x = tf_msg.transform.translation.x;
    position.y = tf_msg.transform.translation.y;
    position.z = tf_msg.transform.translation.z;

    // position.x = odom_msg.pose.pose.position.x;
    // position.y = odom_msg.pose.pose.position.y;
    // position.z = odom_msg.pose.pose.position.z;

    // passthrough filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(position.x - 5.0, position.x + 5.0);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(position.y - 5.0, position.y + 5.0);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(position.z - 5.0, position.z);
    pass.filter(*cloud_pt);


    if(cloud_pt->empty()){
        cloud_pt = cloud;
        ROS_WARN("PassThrough result is empty. Used point cloud without PassThrough Fİlter");
    }
        
    //voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud_pt);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel_grid.filter(*cloud_pt);

    // Estimate surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    ne.setInputCloud(cloud_pt);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod(tree);
    ne.setKSearch(1000);  // Number of nearest neighbors to use for estimating surface normals
    ne.compute(*normals);

    // Create a new point cloud with XYZRGBNormal data type
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    cloud_with_normals->reserve(cloud_pt->size());
    pcl::concatenateFields(*cloud_pt, *normals, *cloud_with_normals);

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud(cloud_with_normals);
    sor.setMeanK(1000);         // The number of points around a reference to determine whether a point is out of group
    sor.setStddevMulThresh(0.05);   // Standard deviation threshold
    sor.filter(*cloud_with_normals_filtered);

    // Filter points with z value less than 2
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_filtered_final(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    for (const auto& point : cloud_with_normals_filtered->points) {
        if (point.z >= 1.75) {
            cloud_with_normals_filtered_final->points.push_back(point);
        }
    }

    // Set the appropriate width and height for the final filtered cloud
    cloud_with_normals_filtered_final->width = cloud_with_normals_filtered_final->points.size();
    cloud_with_normals_filtered_final->height = 1;
    cloud_with_normals_filtered_final->is_dense = true;

    publishNegativeVolume(cloud_with_normals_filtered_final);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_with_normals_filtered_final, output);

    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "negative_volume");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

    ros::Rate rate(5);

    tfSub = nh.subscribe<tf2_msgs::TFMessage>(TF_TOPIC, 1, tfCallBack);
    sub = nh.subscribe<sensor_msgs::PointCloud2>(IMAGE_TOPIC, 1, callBack);

    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    negativeVolumePub = nh.advertise<nav_msgs::OccupancyGrid>(NEGATIVE_VOLUME_TOPIC, 1);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
