#! /bin/bash

ln -s /usr/bin/python3 /usr/bin/python

source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh

mkdir -p /home/catkin_ws/src
cd /home/catkin_ws/src/
catkin_init_workspace
cd /home/catkin_ws/
catkin_make

source /home/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/catkin_ws/src/kangal/kangal_world/models

nohup roslaunch main_package run_simulation.launch gui:=false

source /usr/share/gazebo/setup.sh

sh /src/rootfs/startup.sh &

roslaunch kangal_world gazebo.launch &
sleep 13

# run sensor fusion node
roslaunch main_package sensor_fusion.launch &
sleep 5

# run code node
roslaunch main_package code.launch &
sleep 7

# run move base node
roslaunch main_package move_base.launch 
