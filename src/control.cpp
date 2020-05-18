#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include "mav_msgs/Actuators.h"
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Path.h>

#include "../include/quad_control/planner.h"

class QUAD_CTRL {
    public:
        QUAD_CTRL();
        void odom_cb( nav_msgs::OdometryConstPtr );
        void run();
        void ctrl_loop();
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Publisher _cmd_vel_pub;
        Vector3d _P;
        Vector3d _P_dot;
        Vector3d _Eta;
        Vector3d _Eta_dot;
};

QUAD_CTRL::QUAD_CTRL() {
    _odom_sub = _nh.subscribe("/hummingbird/ground_truth/odometry", 0, &QUAD_CTRL::odom_cb, this);
    _cmd_vel_pub = _nh.advertise< mav_msgs::Actuators>("/hummingbird/command/motor_speed", 0);

    _P.resize(3);
    _Eta.resize(3);
}

void QUAD_CTRL::odom_cb( nav_msgs::OdometryConstPtr odom ) {

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    tf::Matrix3x3 Rb(q);
    Rb.getRPY(_Eta(0), _Eta(1), _Eta(2));

    _P(0) = odom->pose.pose.position.x;
    _P(1) = odom->pose.pose.position.y;
    _P(2) = odom->pose.pose.position.z;


}

void QUAD_CTRL::ctrl_loop() {

  ros::Rate r(100);
  mav_msgs::Actuators comm;
  comm.angular_velocities.resize(4);

  ROS_INFO("Controllo attivo");

  while( ros::ok() ) {
    //ROS_INFO("CLIK plugin started!");
    comm.header.stamp = ros::Time::now();
    for (int i=0; i<4; i++) {
      comm.angular_velocities[i] = 450;
    }

    _cmd_vel_pub.publish (comm);
    r.sleep();
  }

}


void QUAD_CTRL::run() {
    boost::thread ctrl_loop_t ( &QUAD_CTRL::ctrl_loop, this);
}


int main( int argc, char** argv) {

    ros::init(argc, argv, "quad_controller" );

    double limits[6]={-1,6,-1,6,0,5};

    QUAD_PLAN p(limits);
    QUAD_CTRL c;

    ROS_INFO("Programma attivo");
    p.run();
    c.run();

    double init[] = {0,0,0,0};
    double goal[] = {3,3,3,M_PI};
    p.setGoal(init, goal);

    ros::spin();


    return 0;

}
