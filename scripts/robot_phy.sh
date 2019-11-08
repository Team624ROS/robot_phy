#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/robot_phy/devel/setup.bash

# Start roscore and wait till its finished
roscore -p 11311 &
sleep 5
 
# Start realsense
roslaunch realsense2_camera rs_d400_and_t265.launch topic_odom_in:=/odom serial_no_camera1:=925122110604 camera1:=t265 serial_no_camera2:=927522071627 camera2:=d435 publish_odom_tf:=false