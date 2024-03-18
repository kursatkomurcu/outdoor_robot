# Husky
### ROS Packages for Husky Robot

![](https://github.com/Tinker-Twins/Husky/blob/main/Husky.png)

## Dependencies

 - fath_pivot_mount_description: `$ sudo apt-get install ros-noetic-fath-pivot-mount-description`
 - flir_camera_description: `$ sudo apt-get install ros-noetic-flir-camera-description`
 - velodyne_description: `$ sudo apt-get install ros-noetic-velodyne-description`
 - LMS1xx: `$ sudo apt-get install ros-noetic-lms1xx`
 - robot_localization: `$ sudo apt-get install ros-noetic-robot-localization`
 - interactive_marker_twist_server: `$ sudo apt-get install ros-noetic-interactive-marker-twist-server`
 - twist_mux: `$ sudo apt-get install ros-noetic-twist-mux`
 - teleop_twist_keyboard: `$ sudo apt-get install ros-noetic-teleop-twist-keyboard`
 - teleop_twist_joy: `$ sudo apt-get install ros-noetic-teleop-twist-joy`
 - rviz_imu_plugin: `$ sudo apt-get install ros-noetic-rviz-imu-plugin`
 - gmapping: `$ sudo apt-get install ros-noetic-gmapping`

## Installation

1. Create a Catkin workspace (if not already present)
  ```bash
  $ mkdir -p catkin_ws/src
  ```
2. Change directory to the source space (`src`) of your Catkin workspace
  ```bash
  $ cd catkin_ws/src
  ```
3. Clone this repository:
  ```bash
  $ git clone https://github.com/kursatkomurcu/outdoor_robot/tree/kangal_autonomous_ground_vehicle
  ```
4. Change directory back to the Catkin workspace:
  ```bash
  $ cd ..
  ```
5. Build the packages:
  ```bash
  catkin_make

  source devel/setup.bash

  roslaunch run_simulation.launch
  ```
    
# Final Results

![](https://github.com/kursatkomurcu/outdoor_robot/blob/kangal_autonomous_ground_vehicle/kangal.jpeg)

![](https://github.com/kursatkomurcu/outdoor_robot/blob/kangal_autonomous_ground_vehicle/map.jpeg)
