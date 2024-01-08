#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_localization/navsat_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static const std::string GPS_TOPIC = "/move_base_simple/goal_fix";
static const std::string PUBLISH_TOPIC_UTM = "move_base_simple/goal_utm";
static const std::string PUBLISH_TOPIC_MAP = "move_base_simple/goal";
static const std::string PUBLISH_TOPIC_GPS = "move_base_simple/gps";
static const std::string MAP_FRAME = "map";
static const std::string UTM_FRAME = "utm";

ros::Publisher pub_utm, pub_map, pub_gps;
ros::Subscriber sub;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListenerPtr;

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // Print the received GPS data
    ROS_INFO("Latitude: %f", msg->latitude);
    ROS_INFO("Longitude: %f", msg->longitude);
    ROS_INFO("Altitude: %f", msg->altitude);

    // Convert GPS coordinates to UTM coordinates
    double northing;
    double easting;
    RobotLocalization::NavsatConversions::UTM(msg->latitude, msg->longitude, &easting, &northing);

    // Create a move_base_simple/goal message in UTM frame
    geometry_msgs::PoseStamped utmPose;
    utmPose.header.frame_id = UTM_FRAME;
    utmPose.header.stamp = ros::Time::now();
    utmPose.pose.position.x = easting;
    utmPose.pose.position.y = northing;
    utmPose.pose.orientation.w = 1.0; // Assuming no rotation

    // Publish the goal in UTM frame
    pub_utm.publish(utmPose);

    try {
        // Create a static transform between UTM and MAP frames
        static tf2_ros::StaticTransformBroadcaster staticBroadcaster;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = UTM_FRAME;
        transformStamped.child_frame_id = "robot_utm";
        transformStamped.transform.translation.x = easting;
        transformStamped.transform.translation.y = northing;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;
        staticBroadcaster.sendTransform(transformStamped);

        // Transform the UTM pose to the map frame
        geometry_msgs::TransformStamped mapTransformStamped;
        try {
            tfBuffer.transform(transformStamped, mapTransformStamped, MAP_FRAME);
        } catch(tf2::TransformException& ex) {
            ROS_WARN("Failed to transform transformStamped to map frame: %s", ex.what());
            return;
        }

        // Create a map frame PoseStamped message
        geometry_msgs::PoseStamped mapPose;
        mapPose.header.frame_id = MAP_FRAME;
        mapPose.header.stamp = ros::Time::now();
        
        mapPose.pose.position.x = mapTransformStamped.transform.translation.x;
        mapPose.pose.position.y = mapTransformStamped.transform.translation.y;
        mapPose.pose.position.z = mapTransformStamped.transform.translation.z;
        mapPose.pose.orientation.w = 1.0;

        // Publish the transformed pose in the map frame
        pub_map.publish(mapPose);

        ROS_INFO("Transformed pose (map frame): x=%.2f, y=%.2f, z=%.2f", mapPose.pose.position.x, mapPose.pose.position.y, mapPose.pose.position.z);

        MoveBaseClient ac("move_base", true);
    
        ROS_INFO("Waiting for the move_base action server to become available...");
        ac.waitForServer(ros::Duration(5));
        ROS_INFO("move_base action server is now available.");

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = MAP_FRAME;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = mapPose.pose.position.x;
        goal.target_pose.pose.position.y = mapPose.pose.position.y;
        goal.target_pose.pose.orientation.w = 1.0;

        // ROS_INFO("Sending goal...");
        // ac.sendGoal(goal);
        // ac.waitForResult();

        // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        //     ROS_INFO("Robot has reached the goal!");
        // }
        // else{
        //     ROS_WARN("The robot failed to reach the goal...");
        // }

 	ROS_INFO("Sending goal...");
	ac.sendGoal(goal);

       
    } catch(tf2::TransformException& ex) {
        ROS_WARN("Failed to transform pose: %s", ex.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gps");

    ros::NodeHandle nh;
    sub = nh.subscribe(GPS_TOPIC, 1, callback);

    pub_utm = nh.advertise<geometry_msgs::PoseStamped>(PUBLISH_TOPIC_UTM, 10);
    pub_map = nh.advertise<geometry_msgs::PoseStamped>(PUBLISH_TOPIC_MAP, 10);
    pub_gps = nh.advertise<sensor_msgs::NavSatFix>(PUBLISH_TOPIC_GPS, 10);

    // Create the TF2 listener using the globally declared tfBuffer
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);

    ros::Rate rate(10); // Adjust the rate as needed
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    {
	sensor_msgs::NavSatFix navsatFixMsg;
	navsatFixMsg.header.stamp = ros::Time::now();
	navsatFixMsg.status.status = 0;
	navsatFixMsg.status.service = 0;
	navsatFixMsg.latitude = 39.94852465525221;
	navsatFixMsg.longitude = 32.769577303459414;
	navsatFixMsg.altitude = 0.0;
	

	ROS_INFO("Latitude: %f", navsatFixMsg.latitude);
	ROS_INFO("Longitude: %f", navsatFixMsg.longitude);
	ROS_INFO("Altitude: %f", navsatFixMsg.altitude);

	// Convert GPS coordinates to UTM coordinates
	double northing;
	double easting;
	RobotLocalization::NavsatConversions::UTM(navsatFixMsg.latitude, navsatFixMsg.longitude, &easting, &northing);

	// Create a move_base_simple/goal message in UTM frame
	geometry_msgs::PoseStamped utmPose;
	utmPose.header.frame_id = UTM_FRAME;
	utmPose.header.stamp = ros::Time::now();
	utmPose.pose.position.x = easting;
	utmPose.pose.position.y = northing;
	utmPose.pose.orientation.w = 1.0; // Assuming no rotation

	// Publish the goal in UTM frame
	pub_utm.publish(utmPose);

	try {
		// Create a static transform between UTM and MAP frames
		tf2_ros::StaticTransformBroadcaster staticBroadcaster;
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = UTM_FRAME;
		transformStamped.child_frame_id = "robot_utm";
		transformStamped.transform.translation.x = easting;
		transformStamped.transform.translation.y = northing;
		transformStamped.transform.translation.z = 0.0;
		transformStamped.transform.rotation.x = 0.0;
		transformStamped.transform.rotation.y = 0.0;
		transformStamped.transform.rotation.z = 0.0;
		transformStamped.transform.rotation.w = 1.0;
		staticBroadcaster.sendTransform(transformStamped);

		// Transform the UTM pose to the map frame
		geometry_msgs::TransformStamped mapTransformStamped;
		try {
			
			tfBuffer.canTransform(MAP_FRAME, UTM_FRAME,
                      					ros::Time::now(), ros::Duration(60.0));
			tfBuffer.transform(transformStamped, mapTransformStamped, MAP_FRAME);
		} catch(tf2::TransformException& ex) {
			ROS_WARN("Failed to transform transformStamped to map frame: %s", ex.what());
			
		}

		// Create a map frame PoseStamped message
		geometry_msgs::PoseStamped mapPose;
		mapPose.header.frame_id = MAP_FRAME;
		mapPose.header.stamp = ros::Time::now();

		mapPose.pose.position.x = mapTransformStamped.transform.translation.x;
		mapPose.pose.position.y = mapTransformStamped.transform.translation.y;
		mapPose.pose.position.z = mapTransformStamped.transform.translation.z;
		mapPose.pose.orientation.w = 1.0;

		// Publish the transformed pose in the map frame
		pub_map.publish(mapPose);

		ROS_INFO("Transformed pose (map frame): x=%.2f, y=%.2f, z=%.2f", mapPose.pose.position.x, mapPose.pose.position.y, mapPose.pose.position.z);

		MoveBaseClient ac("move_base", true);

		ROS_INFO("Waiting for the move_base action server to become available...");
		ac.waitForServer(ros::Duration(60));
		ROS_INFO("move_base action server is now available.");

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = MAP_FRAME;
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = mapPose.pose.position.x;
		goal.target_pose.pose.position.y = mapPose.pose.position.y;
		goal.target_pose.pose.orientation.w = 1.0;

		// ROS_INFO("Sending goal...");
		// ac.sendGoal(goal);
		// ac.waitForResult();

		// if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		//     ROS_INFO("Robot has reached the goal!");
		// }
		// else{
		//     ROS_WARN("The robot failed to reach the goal...");
		// }

		ROS_INFO("Sending goal...");
		ac.sendGoal(goal);


	} catch(tf2::TransformException& ex) {
		ROS_WARN("Failed to transform pose: %s", ex.what());
	}
    }
    
    
    ros::waitForShutdown();

    delete tfListenerPtr;

    return 0;
}

/*
Enlem Boylam Yükseklik Değerleri:

latitude: 49.899768316396944
longitude: 8.89996366856024
altitude: -1.00529439851056

latitude: 49.89977838346621
longitude: 8.900071268506576
altitude: -0.9636453417378488

latitude: 49.89976957559654
longitude: 8.90000548264246
altitude: -1.0112892212547795

latitude: 49.89973438395983
longitude: 8.899942380331913
altitude: -0.8263795033987708

rostopic pub --once /move_base_simple/goal_fix sensor_msgs/NavSatFix "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
status: {status: 0, service: 0}
latitude: 49.900712660524746
longitude: 8.900719770176472
altitude: -1.2020334709814404
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

## Hedef tepe arkası
latitude: 49.900712660524746
longitude: 8.900719770176472
altitude: -1.2020334709814404

# düz yol
latitude: 49.900053944444075
longitude: 8.899972163781616
altitude: 0.18231535550524003


*/
