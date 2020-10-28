# Quadrotor Library
The ned_plugin folder contains the source code for the ned_conv gazebo world plugin used to convert the ENU odometry and Force/Torque commands into NED ones. This plugin is already included in the provided arena.world file. You can put your code inside the quad_control folder. All the other folders are from the [RotorS MAV simulator](https://github.com/ethz-asl/rotors_simulator) and provide the dynamic Gazebo model of the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/) quadcopter.

## Prerequisites
You will need all the packages on which the RotorS library already depends.
For the mathematical computation, is also needed the Eigen library http://eigen.tuxfamily.org/.

## Installation instructions - Ubuntu 18.04 with ROS Melodic
---------------------------------------------------------
 1. Create a ROS workspace and clone this repository inside your_workspace/src folder. Compile all these ros packages inside the workspace with the following command (you may need to install others ROS dependencies if some errors appears during the compilation):
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

 3. The two most important reference frames are **worldNED** and **hummingbird/base_linkNED**. The goal of the project is to generate a trajectory and control the quadrotor in order to move the **hummingbird/base_linkNED** (attached to the quadrotor body) with respect to **worldNED** (the fixed frame in the world).

 4. As an interface with the Gazebo simulation, you should use two topics:
    * /hummingbird/ground_truth/odometryNED: from here you can get all the odometry informations you need about the quadrotor (position and velocity), in particular those related to the **hummingbird/base_linkNED** frame. The linear and angular positions are with respect to the world frame (worldNED). The linear and angular velocities are with respect to the **body frame**, remember that when using these data.
    * /hummingbird/command/wrenchNED: on this topic you should publish the command forces and torques you want to apply on the quadrotor body that your control algorithm produces. This topic acceptes an entire wrench as input, but remember that **you cannot apply linear forces along the x and y directions, so those forces will not be considered by the quadrotor**. You have to use the z-component of the linear force and the 3 torques to control the quadrotor.

5. You can create your flying controller and planner in the quad_control folder. Just adjust the CMake file to your needs.


## Pro Tips
 -----------

  1. Once you have created a trajectory planner, you can easily check your trajectory by publishing it with a **nav_msgs/Path** message type. This kind of message can be printed from rviz and provides a useful visual feedback to check if the planner is working as expected.

  2. You can record all the topics in your simulation in a bagfile with the following command:
     ```
     $ rosbag record -a -O bagFileName
     ```

  3. Once you have recorded the bagfile, you can plot all the collected data with **rqt_bag**. This can be useful to debug your applications and to obtain the plots for your technical report.
     ```
     $ rosrun rqt_bag rqt_bag
     ```
