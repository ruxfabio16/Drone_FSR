# Quadrotor Library
The ned_plugin folder contains the source code for the ned_conv gazebo world plugin used to convert the ENU odometry and Force/Torque commands into NED ones. All the other folders are from the [RotorS MAV simulator](https://github.com/ethz-asl/rotors_simulator) and provide the dynamic Gazebo model of the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/) quadcopter.

## Prerequisites
You will need all the packages on which the RotorS library already depends.
For the mathematical computation, is also needed the Eigen library http://eigen.tuxfamily.org/.

## Installation instructions - Ubuntu 18.04 with ROS Melodic
---------------------------------------------------------
 1. Create a ROS workspace and compile all these folders inside the workspace with the command (you may need to install others ROS dependencies if some errors appears during the compilation):
 ```
 $ catkin_make
 ```

## Usage and testing
 -----------

 1. Use the provided launchfile to open the gazebo scene and Rviz:
 ```
 $ roslaunch quad_control mymav.launch
 ```

 2. You will need to create your flying arena in Gazebo. To do that, you can just modify the "arena.world" file inside "quad_control/worlds".

 3. As an interface with the Gazebo simulation, among the various topics, you can use two: "/hummingbird/ground_truth/odometryNED" and "/hummingbird/command/wrenchNED".
