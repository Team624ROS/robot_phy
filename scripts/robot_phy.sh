#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/robot_phy/devel/setup.bash

# Start roscore and wait till its finished
roscore -p 11311 &
sleep 5

# Start realsense  # CHANGE CALB FILE PATH FOR ODOM
roslaunch rs_launcher rs_d400_and_t265.launch serial_no_camera1:=925122110604 camera1:=t265 serial_no_camera2:=927522071627 camera2:=d435 &
#roslaunch rs_launcher rs_t265.launch calib_odom_file:=/home/team624/robot_phy/src/rs_launcher/calibration/calibration_odometry.json topic_odom_in:=/odom serial_no_camera1:=925122110604 camera1:=t265

chmod 666 /dev/rplidarUSB
roslaunch rplidar rplidar.launch
