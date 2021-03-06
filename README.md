# Quadrotor Library
The *ned_plugin* folder contains the source code for the *ned_conv* gazebo world plugin used to convert the ENU odometry and Force/Torque commands into NED ones. This plugin is already included in the provided *arena.world* file.  
You can put your code (the planner and the controller that you will create) inside the *quad_control* folder.  
All the other folders are from the [RotorS MAV simulator](https://github.com/ethz-asl/rotors_simulator) and provide the dynamic Gazebo model of the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/) quadcopter.

## Prerequisites
You will need all the packages on which the RotorS library already depends.
For the mathematical computation, is also needed the Eigen library http://eigen.tuxfamily.org/.

## Installation instructions - Ubuntu 18.04 with ROS Melodic
 1. Create a ROS workspace and clone this repository inside *your_workspace/src* folder. Make sure that all the folders in this repository are directly inside the *src* folder.  
    Compile all these ros packages inside the workspace with the following command (you may need to install others ROS dependencies if some errors appears during the compilation):
    ```
    $ catkin_make
    ```

## Usage and testing
 1. Use the provided launchfile to open the gazebo scene and Rviz:
    ```
    $ roslaunch quad_control mymav.launch
    ```

 2. You will need to create your flying arena in Gazebo. To do that, you can just modify the *arena.world* file inside *quad_control/worlds* folder.
    - **NB.** If you already have your own .world file working in simulation with the RotorS Hummingbird quadrotor, you can just add inside your .world file, between the `<world></world>` tags, the following line. This is the plugin you obtain when compiling the *ned_plugin* folder. This plugin creates the two topics explained at (4).
    ```
     <plugin name='ned_conversions' filename='libned_conv.so'/>
    ```

 3. The two most important reference frames are **worldNED** and **hummingbird/base_linkNED**. The goal of the project is to generate a trajectory and control the quadrotor in order to move the **hummingbird/base_linkNED** (attached to the quadrotor body) with respect to **worldNED** (the fixed frame in the world).

 4. As an interface with the Gazebo simulation, you should use two topics:
    * */hummingbird/ground_truth/odometryNED*: from here you can get all the odometry informations you need about the quadrotor (position and velocity), in particular those related to the **hummingbird/base_linkNED** frame. The linear and angular positions are with respect to the world frame (worldNED). The linear and angular velocities are with respect to the **body frame**, remember that when using these data.
      - **NB.** If you need the velocity data with respect to the world frame, instead of the body frame, remember that you can premultiply the velocities by the body rotation matrix *R_b*. You can obtain this matrix directly from the orientation quaternion in the *pose* part of the odometry message.
    
    * */hummingbird/command/wrenchNED*: on this topic you should publish the command forces and torques you want to apply on the quadrotor body that your control algorithm produces. This topic acceptes an entire wrench as input, but remember that **you cannot apply linear forces along the x and y directions, so those forces will not be considered by the quadrotor**. You have to use the z-component of the linear force and the 3 torques to control the quadrotor.

5. You can create your flying controller and planner in the quad_control folder. Just adjust the CMake file to your needs.

## Quadrotor parameters
 The only parameters your should need are the Inertia matrix `I=diag(0.007, 0.007, 0.012) Kg m^2` and the quadrotor mass `m=0.68 Kg`. Since you can control the quadrotor directly with a wrench command, you don't need the allocation matrix.

## Pro Tips

  1. Once you have created a trajectory planner, you can easily check your trajectory by publishing it with a **nav_msgs/Path** message type. This kind of message can be printed from rviz and provides a useful visual feedback to check if the planner is working as expected.

  2. You can record all the topics in your simulation in a bagfile with the following command:
     ```
     $ rosbag record -a -O bagFileName
     ```

  3. Once you have recorded the bagfile, you can plot all the collected data with **rqt_bag**. This can be useful to debug your applications and to obtain the plots for your technical report.
     ```
     $ rosrun rqt_bag rqt_bag
     ```

## Acknowledgements
The code in the folders *ned_plugin* and *quad_control* has been created by [Eugenio Cuniato](https://github.com/ecuniato) under the supervision of [Fabio Ruggiero](http://www.fabioruggiero.name/web/index.php/en) and [Jonathan Cacace](http://wpage.unina.it/jonathan.cacace/).
