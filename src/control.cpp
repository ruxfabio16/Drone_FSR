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
        Vector3d _wbb;
        //Vector3d _wb;
        Matrix3d _Q;
        Matrix4d _G;
        double _c_T;
        double _c_a;
        double _l;
        double _m;
        Matrix3d _I_b;
};

QUAD_CTRL::QUAD_CTRL() {
    _odom_sub = _nh.subscribe("/hummingbird/ground_truth/odometry", 0, &QUAD_CTRL::odom_cb, this);
    _cmd_vel_pub = _nh.advertise< mav_msgs::Actuators>("/hummingbird/command/motor_speed", 0);

    _P.resize(3);
    _Eta.resize(3);

    _l = 0.17; //meters
    _c_T = 8.06428e-05;
    _c_a = 0.000001;
    _m = 0.68; //Kg

    //Inertia matrix
    _I_b << 0.007, 0, 0,
           0, 0.007, 0,
           0, 0, 0.012;

    _G(0,0) = _c_T;    _G(0,1) = _c_T;    _G(0,2) = _c_T; _G(0,3) = _c_T;
    _G(1,0) = 0;       _G(1,1) = _l*_c_T; _G(1,2) = 0;    _G(1,3) = -_l*_c_T;
    _G(2,0) = -_l*_c_T; _G(2,1) = 0;       _G(2,2) = _l*_c_T; _G(2,3) = 0;
    _G(3,0) = _c_a;    _G(3,1) = _c_a;    _G(3,2) = _c_a; _G(3,3) = _c_a;

}

void QUAD_CTRL::odom_cb( nav_msgs::OdometryConstPtr odom ) {

    tf::Matrix3x3 RNed;
    RNed.setEulerYPR(M_PI/2,0,M_PI);
    //RNed = RNed.transpose();
    tf::Vector3 p;
    tf::Vector3 pDot;
    tf::Vector3 wbb;
    tf::Vector3 wb;
    tf::Vector3 etaDot;

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    tf::Matrix3x3 Rb(q);
    tf::Matrix3x3 RbNed = RNed*Rb*(RNed.transpose());

    RbNed.getRPY(_Eta(0), _Eta(1), _Eta(2)); //phi theta psi

    p[0] = odom->pose.pose.position.x;
    p[1] = odom->pose.pose.position.y;
    p[2] = odom->pose.pose.position.z;

    p = RNed*p;
    _P(0) = p[0];
    _P(1) = p[1];
    _P(2) = p[2];

    _Q(0,0) = 1; _Q(0,1) = 0;             _Q(0,2) = -sin(_Eta(1));
    _Q(1,0) = 0; _Q(1,1) = cos(_Eta(0));  _Q(1,2) = cos(_Eta(1))*sin(_Eta(0));
    _Q(2,0) = 0; _Q(2,1) = -sin(_Eta(0)); _Q(2,2) = cos(_Eta(1))*cos(_Eta(0));

    pDot[0] = odom->twist.twist.linear.x;
    pDot[1] = odom->twist.twist.linear.y;
    pDot[2] = odom->twist.twist.linear.z;
    //pDot = RNed*Rb*pDot;
    pDot = RbNed*pDot;

    _P_dot(0) = pDot[0];
    _P_dot(1) = pDot[1];
    _P_dot(2) = pDot[2];

    wbb[0] = odom->twist.twist.angular.x;
    wbb[1] = odom->twist.twist.angular.y;
    wbb[2] = odom->twist.twist.angular.z;

    _wbb(0) = wbb[0];
    _wbb(1) = wbb[1];
    _wbb(2) = wbb[2];

    wb = RbNed * wbb;
    _Eta_dot = _Q.inverse() * _wbb;


    ROS_INFO("x: %f  y: %f z: %f",_P_dot(0),_P_dot(1),_P_dot(2));
    //ROS_INFO("x: %f  y: %f z: %f",_P(0),_P(1),_P(2));
    ROS_INFO("phi: %f  tetha: %f psi: %f",_Eta(0)*180.0/M_PI,_Eta(1)*180.0/M_PI,_Eta(2)*180.0/M_PI);
}

void QUAD_CTRL::ctrl_loop() {

  ros::Rate r(100);
  mav_msgs::Actuators comm;
  comm.angular_velocities.resize(4);

  /* Red: 0 poi gli 1-2-3 in senso antiorario
     0 frontale asse x - 1 frontale asse y
     NED: 1 frontale asse x - 0 frontale asse y
          w1 = 1, w4 = 2, w3 = 3, w2 = 0*/

  ROS_INFO("Controllo attivo");

  while( ros::ok() ) {



    comm.header.stamp = ros::Time::now();
    //for (int i=0; i<4; i++) {
      comm.angular_velocities[1] = 450;
      comm.angular_velocities[2] = 450;
      comm.angular_velocities[3] = 450;
      comm.angular_velocities[0] = 0;
    //}

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
