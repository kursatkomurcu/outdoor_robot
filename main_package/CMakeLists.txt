cmake_minimum_required(VERSION 3.0.2)
project(main_package)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  message_filters
  nav_msgs
  rospy
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  robot_localization
  laser_geometry
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



catkin_package(
 INCLUDE_DIRS include
  LIBRARIES main_package
  CATKIN_DEPENDS geometry_msgs image_transport message_filters nav_msgs roscpp sensor_msgs pcl_conversions pcl_ros robot_localization tf laser_geometry
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

add_executable(estimating_surface_normals_depthCam src/estimating_surface_normals_depthCam.cpp)
target_link_libraries(estimating_surface_normals_depthCam ${catkin_LIBRARIES})

add_executable(point_cloud_derivative src/point_cloud_derivative.cpp)
target_link_libraries(point_cloud_derivative ${catkin_LIBRARIES})

add_executable(gps src/gps.cpp)
target_link_libraries(gps ${catkin_LIBRARIES})

add_executable(negative_volume src/negative_volume.cpp)
target_link_libraries(negative_volume ${catkin_LIBRARIES})

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

add_executable(ransac_segmentation src/ransac_segmentation.cpp)
target_link_libraries(ransac_segmentation ${catkin_LIBRARIES})

add_custom_target(gui
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/gui.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
  COMMENT "Running GUI Python script"
)

# Link any necessary dependencies to the custom target
add_dependencies(gui
  ${catkin_EXPORTED_TARGETS}
)
#target_link_libraries(${PROJECT_NAME}
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES})
  
# add_dependencies(${PROJECT_NAME} sensor_msgs_gencpp)
