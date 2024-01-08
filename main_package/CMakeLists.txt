cmake_minimum_required(VERSION 3.0.2)
project(main_package)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  robot_localization
  tf2_ros
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



catkin_package(
 INCLUDE_DIRS include
  LIBRARIES main_package
  CATKIN_DEPENDS geometry_msgs image_transport message_filters nav_msgs roscpp sensor_msgs pcl_conversions pcl_ros robot_localization tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME} src/rgbd_odom_driver.cpp src/rgbd_odom.cpp)
add_executable(estimating_surface_normals_pointCloud src/estimating_surface_normals_pointCloud.cpp)
target_link_libraries(estimating_surface_normals_pointCloud ${catkin_LIBRARIES})

add_executable(gps src/gps.cpp)
target_link_libraries(gps ${catkin_LIBRARIES})

add_executable(negative_volume src/negative_volume.cpp)
target_link_libraries(negative_volume ${catkin_LIBRARIES})

#target_link_libraries(${PROJECT_NAME}
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES})
  
# add_dependencies(${PROJECT_NAME} sensor_msgs_gencpp)
