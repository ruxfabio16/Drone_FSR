# Quadrotor FSR
Project C2-P2. FSR exam - Eugenio Cuniato.

## Prerequisites

The rotors project is fundamental for the simulation and a copy of it is already included in this repository. The planner and controller developed by me are inside the quad_control folder.

To compile this project you need the fcl library from https://github.com/flexible-collision-library/fcl. This can be simply installed with:
 ```
sudo apt-get install ros-melodic-fcl-catkin
```

You will also need the Octomap package http://octomap.github.io/. This can be simply installed with:
 ```
sudo apt-get ros-melodic-octomap
 ```

For the mathematical computation, is also needed the Eigen library http://eigen.tuxfamily.org/.

## Installation

To install this package, just clone this repository inside a ROS workspace and compile using:
 ```
catkin_make
 ```

## Usage and testing

Start the simulation by launching:
```
roslaunch quad_control mymav.launch gui:=true
```

(Optional) Once Gazebo and Rviz have started, publish the arena map with:
```
rosrun octomap_server octomap_server_node path/to/quad_control/arena.bt

```
The map should now appear in Rviz.


Start the quadrotor controller:
```
rosrun quad_control quad_controller
```
