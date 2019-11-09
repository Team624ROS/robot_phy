# robot_phy
This is the workspace where nodes controling the physical aspects of the robot are stored

The packages of this workspace require that you set up, download dependent packages, and clone the realsense-ros package into the src directory https://github.com/IntelRealSense/realsense-ros#installation-instructions (Follow the instructions)

(This includes also cloning the https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel package into the src)

Use sudo apt-get install ros-kinetic-rplidar-ros to get the rplidar package

NOTE: THIS METHOD MAY ONLY WORK FOR UBUNTU
Need to set up USB ports: Change directory to the /etc/udev/rules.d and open the file under it with sudo then add the text:    ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidarUSB"

NOTE: You must replace the values for idproduct and vendor with the correct info you can find this by plugging in the device and running (usb-devices) (run this with and without the sensor pluged in so you can tell which device it is) Change the SYMLINK to the name you want.

Check if the usb is working and connected with (ls /dev/rplidarUSB) 

