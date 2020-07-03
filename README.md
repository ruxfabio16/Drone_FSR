# Quadrotor FSR
Project C2-P2. FSR exam - Eugenio Cuniato.

The developed controller is inside the "quad_control" folder. All the other folders are from the [RotorS MAV simulator](https://github.com/ethz-asl/rotors_simulator) and provide the dynamic Gazebo model of the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/) quadcopter.

## Prerequisites
To compile this project you need the fcl library from https://github.com/flexible-collision-library/fcl. This can be simply installed with:
 ```
sudo apt-get install ros-melodic-fcl-catkin
```

You will also need the Octomap package http://octomap.github.io/. This can be simply installed with:
 ```
sudo apt-get ros-melodic-octomap
 ```

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
 $ roslaunch quad_control mymav.launch gui:=true
 ```

 2. Launch the octomap server in order to see the obstacles in Rviz:
 ```
 $ rosrun octomap_server octomap_server_node /path/to/quad_control/arena.bt
 ```

 3. Launch the quadcopter controller to have it moving between different waypoints and finally land:
 ```
 $ rosrun quad_control quad_controller
 ```
