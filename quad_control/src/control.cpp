#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include "mav_msgs/Actuators.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Path.h>

#include "../include/quad_control/planner.h"

double g = 9.81;

Matrix3d Skew(Vector3d v) {
  Matrix3d S;
  S << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;

  return S;
}

Vector3d Vee(Matrix3d S) {
  Vector3d v;
  v<<S(2,1),S(0,2),S(1,0);
  //cout<<S<<endl;
  return v;
}


class QUAD_CTRL {
    public:
        QUAD_CTRL();
        ~QUAD_CTRL() {_shutdown=true; ROS_INFO("Shutting down...");};
        void odom_cb( nav_msgs::OdometryConstPtr );
        void run();
        void ctrl_loop();
        void setTraj(std::vector<geometry_msgs::PoseStamped> p, std::vector<geometry_msgs::TwistStamped> v, std::vector<geometry_msgs::AccelStamped> a);
        bool isTrajReady() {return _traj_ready;};
        bool isTrajCompleted() {return _traj_completed;};
        bool isLanded() {return _landed;};
        void land() {_isLanding=true;};
        void getPose(double * Pose);
    private:
        void updateError();
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Publisher _cmd_vel_pub;
        Vector3d _P;
        Vector3d _P_dot;
        Vector3d _Eta;
        Vector3d _Eta_dot;
        Vector3d _etaDot_des, _etadd_des;
        Vector3d _wbb;
        Matrix3d _Rb, _Rb_des;
        Matrix3d _RNed;
        Vector3d _Ep;
        Vector3d _Ev;
        Vector3d _Er, _Ew;
        //Vector3d _wb;
        Matrix3d _Q;
        Matrix4d _G;
        double _c_T;
        double _c_a;
        double _l;
        double _m;
        Matrix3d _I_b;
        std::vector<geometry_msgs::PoseStamped> _poses;
        std::vector<geometry_msgs::TwistStamped> _velocities;
        std::vector<geometry_msgs::AccelStamped> _accelerations;
        bool _traj_ready;
        bool _traj_completed;
        bool _isLanding;
        bool _landed;
        bool _odomOk;
        Vector3d _P_des, _Pd_des, _Pdd_des;
        Vector3d _wbb_des, _wbbd_des;
        int _iter;
        double _Kp, _Kv, _Kr, _Kw;
        double _uT;
        Vector3d _tau_b;
        bool _isSettingTraj;
        tf::Quaternion _q_des;
        mav_msgs::Actuators _comm;
        bool _shutdown;
};

void QUAD_CTRL::getPose(double * Pose) {
  Vector3d P;
  Matrix3d Rb;
  tf::Matrix3x3 RbTf;
  double roll, pitch, yaw;

  while(!_odomOk) usleep(100);

  P = _RNed.transpose()*_P;

  Rb = _RNed.transpose()*_Rb*_RNed;
  matrixEigenToTF(Rb, RbTf);
  RbTf.getRPY(roll, pitch, yaw);
  Pose[0] = P(0); Pose[1] = P(1); Pose[2] = P(2); Pose[3] = yaw;
}

void QUAD_CTRL::updateError() {
  Vector3d zb_des;
  Vector3d yb_des;
  Vector3d xb_des;
  Vector3d e3(0,0,1);
  Matrix3d Rb_des;
  Matrix3d Q_des;
  Matrix3d Qd_des;
  Vector3d wbb_des;
  Vector3d etaDot_des;
  Vector3d eta_des;
  Vector3d wbbd_des;
  Vector3d etadd_des;

  //if (!_traj_completed) {
    _P_des[0] = _poses[_iter].pose.position.x;
    _P_des[1] = _poses[_iter].pose.position.y;
    _P_des[2] = _poses[_iter].pose.position.z;

    _P_des = _RNed * _P_des;

    _Pd_des[0] = _velocities[_iter].twist.linear.x;
    _Pd_des[1] = _velocities[_iter].twist.linear.y;
    _Pd_des[2] = _velocities[_iter].twist.linear.z;

    _Pd_des = _RNed*_Pd_des;

    _Pdd_des[0] = _accelerations[_iter].accel.linear.x;
    _Pdd_des[1] = _accelerations[_iter].accel.linear.y;
    _Pdd_des[2] = _accelerations[_iter].accel.linear.z;

    _Pdd_des = _RNed*_Pdd_des;
  //}

  _Ep = _P - _P_des;
  _Ev = _P_dot - _Pd_des;
  //cout<<_Ep<<endl;
  //cout<<_P<<endl;
  //cout<<_Pdd_des<<endl;
  zb_des = _Kp*_Ep + _Kv*_Ev + _m*g*e3 - _m*_Pdd_des;
  _uT = zb_des.transpose() * _Rb * e3;
  zb_des = zb_des/zb_des.norm();

  //if(!_traj_completed) {
    tf::Quaternion q_des(_poses[_iter].pose.orientation.x, _poses[_iter].pose.orientation.y, _poses[_iter].pose.orientation.z,  _poses[_iter].pose.orientation.w);
    _q_des = q_des;
    _etaDot_des << 0,0,_velocities[_iter].twist.angular.z;
    _etadd_des << 0,0,_accelerations[_iter].accel.angular.z;
  //}
  tf::Matrix3x3 Rb_des_tf(_q_des);
  //Rb_des_tf.getRPY(eta_des(0), eta_des(1), eta_des(2)); //phi theta psi
  tf::matrixTFToEigen(Rb_des_tf,Rb_des);

  Rb_des = _RNed*Rb_des*(_RNed.transpose()); //Rb NED transform

  xb_des = Rb_des.col(0);
  yb_des = zb_des.cross(xb_des);
  yb_des = yb_des / yb_des.norm();
  xb_des = yb_des.cross(zb_des);
  Rb_des << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);

  _Rb_des = Rb_des;


  Matrix3d Rb_des_noNED = (_RNed.transpose())*Rb_des*_RNed;
  tf::Matrix3x3 Rb_des_noNED_tf;
  tf::matrixEigenToTF(Rb_des_noNED,Rb_des_noNED_tf);
  Rb_des_noNED_tf.getRPY(eta_des(0), eta_des(1), eta_des(2)); //phi theta psi

  Q_des(0,0) = 1; Q_des(0,1) = 0;                Q_des(0,2) = -sin(eta_des(1));
  Q_des(1,0) = 0; Q_des(1,1) = cos(eta_des(0));  Q_des(1,2) = cos(eta_des(1))*sin(eta_des(0));
  Q_des(2,0) = 0; Q_des(2,1) = -sin(eta_des(0)); Q_des(2,2) = cos(eta_des(1))*cos(eta_des(0));

  wbb_des = Q_des * _etaDot_des;
  Vector3d appo(wbb_des(1),wbb_des(0),-wbb_des(2));
  wbb_des = appo;
  _wbb_des = wbb_des;
  //wbb_des << 0,0,0;

  Qd_des(0,0) = 0; Qd_des(0,1) = 0;                               Qd_des(0,2) = -cos(eta_des(1))*_etaDot_des(1);
  Qd_des(1,0) = 0; Qd_des(1,1) = -sin(eta_des(0))*_etaDot_des(0);  Qd_des(1,2) = -sin(eta_des(1))*_etaDot_des(1)*sin(eta_des(0)) + cos(eta_des(1))*_etaDot_des(0)*cos(eta_des(0));
  Qd_des(2,0) = 0; Qd_des(2,1) = -cos(eta_des(0))*_etaDot_des(0);  Qd_des(2,2) = -sin(eta_des(1))*_etaDot_des(1)*cos(eta_des(0)) - cos(eta_des(1))*_etaDot_des(0)*sin(eta_des(0));

  wbbd_des = Qd_des * _etaDot_des + Q_des * _etadd_des;
  Vector3d appo1(wbbd_des(1),wbbd_des(0),-wbbd_des(2));
  wbbd_des = appo1;
  _wbbd_des = wbbd_des;

  _Er = 0.5*Vee(_Rb_des.transpose()*_Rb - _Rb.transpose()*_Rb_des);
  _Ew = _wbb - _Rb.transpose()*_Rb_des*_wbb_des;

  _tau_b = -_Kr*_Er - _Kw*_Ew + Skew(_wbb)*_I_b*_wbb - _I_b*( Skew(_wbb)*_Rb.transpose()*_Rb_des*_wbb_des - _Rb.transpose()*_Rb_des*_wbbd_des );
  //cout<<_Er<<endl;
  //cout<<_Ep<<endl;
  //cout<<"Meas: "<<_Eta(2)<<" - Des: "<<eta_des(2)<<endl;
  if (_iter < (_poses.size() - 1)) _iter++;
  else if (!_traj_completed) {
    _traj_completed = true;
    _traj_ready = false;
  }

  _odomOk =true;

}

void QUAD_CTRL::setTraj(std::vector<geometry_msgs::PoseStamped> p, std::vector<geometry_msgs::TwistStamped> v, std::vector<geometry_msgs::AccelStamped> a) {

  for (int i=0; i<p.size(); i++) {
    _poses.push_back(p[i]);
    _velocities.push_back(v[i]);
    _accelerations.push_back(a[i]);
  }

  _traj_ready = true;
  _traj_completed = false;
  //_iter = 0;
}

QUAD_CTRL::QUAD_CTRL() {
    _odom_sub = _nh.subscribe("/hummingbird/ground_truth/odometry", 0, &QUAD_CTRL::odom_cb, this);
    _cmd_vel_pub = _nh.advertise< mav_msgs::Actuators>("/hummingbird/command/motor_speed", 0);

    _traj_ready = false;
    _traj_completed = false;
    _isLanding = false;
    _landed = false;
    _odomOk = false;
    _iter = 0;
    _shutdown = false;

    _P.resize(3);
    _Eta.resize(3);

    _l = 0.17; //meters
    _c_T = 8.06428e-06;
    _c_a = -0.000001;
    //_c_a = -0.016;
    _m = 0.68;//0.68; //Kg

    //Inertia matrix
    _I_b << 0.007, 0, 0,
           0, 0.007, 0,
           0, 0, 0.012;

    _G(0,0) = _c_T;    _G(0,1) = _c_T;    _G(0,2) = _c_T; _G(0,3) = _c_T;
    _G(1,0) = 0;       _G(1,1) = _l*_c_T; _G(1,2) = 0;    _G(1,3) = -_l*_c_T;
    _G(2,0) = -_l*_c_T; _G(2,1) = 0;       _G(2,2) = _l*_c_T; _G(2,3) = 0;
    _G(3,0) = -_c_a;    _G(3,1) = _c_a;    _G(3,2) = -_c_a; _G(3,3) = _c_a;

    _Kp = 5;
    _Kv = _Kp/2;
    _Kr = 0.4;
    _Kw = _Kr/5;

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
    tf::matrixTFToEigen (RbNed, _Rb);
    tf::matrixTFToEigen (RNed, _RNed);

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
    //cout<<odom->twist.twist.linear.x;
    pDot = RNed*Rb*pDot;
    //cout<<pDot[0];
    //pDot = RNed*Rb*pDot*RNed.transpose();

    _P_dot(0) = pDot[0];
    _P_dot(1) = pDot[1];
    _P_dot(2) = pDot[2];

    wbb[0] = odom->twist.twist.angular.y;
    wbb[1] = odom->twist.twist.angular.x;
    wbb[2] = -odom->twist.twist.angular.z;

    _wbb(0) = wbb[0];
    _wbb(1) = wbb[1];
    _wbb(2) = wbb[2];

    _Eta_dot = _Q.inverse() * _wbb;

    _odomOk = true;
    //ROS_INFO("x: %f  y: %f z: %f",_P_dot(0),_P_dot(1),_P_dot(2));
    //ROS_INFO("x: %f  y: %f z: %f",_P(0),_P(1),_P(2));
    //ROS_INFO("phi: %f  tetha: %f psi: %f",_Eta(0)*180.0/M_PI,_Eta(1)*180.0/M_PI,_Eta(2)*180.0/M_PI);
}

void QUAD_CTRL::ctrl_loop() {

  ros::Rate r(TimeRate);
  _comm.angular_velocities.resize(4);
  _comm.angular_velocities[0] = 0;
  _comm.angular_velocities[1] = 0;
  _comm.angular_velocities[2] = 0;
  _comm.angular_velocities[3] = 0;
  Vector4d w2, controlInput;


  /* Red: 0 poi gli 1-2-3 in senso antiorario
     0 frontale asse x - 1 frontale asse y
     NED: 1 frontale asse x - 0 frontale asse y
          w1 = 1, w4 = 2, w3 = 3, w2 = 0*/

  ROS_INFO("Controllo attivo");

  while(!isTrajReady() || !_odomOk) {
    _cmd_vel_pub.publish (_comm);
    usleep(1000);
  }

  while( ros::ok() && !_shutdown ) {

    if (!_isLanding) {
      updateError();
      controlInput(0) = _uT;
      controlInput(1) = _tau_b(0);
      controlInput(2) = _tau_b(1);
      controlInput(3) = _tau_b(2);

      w2 = _G.inverse() * controlInput;
//cout<<_Ep<<endl;
      _comm.header.stamp = ros::Time::now();
    /*  _comm.angular_velocities[0] = sqrt(w2(3));
      _comm.angular_velocities[1] = sqrt(w2(2));
      _comm.angular_velocities[2] = sqrt(w2(1));
      _comm.angular_velocities[3] = sqrt(w2(0)); */

      if (w2(0)>=0 && w2(1)>=0 && w2(2)>=0 && w2(3)>=0) {
        _comm.angular_velocities[0] = sqrt(w2(3));
        _comm.angular_velocities[1] = sqrt(w2(2));
        _comm.angular_velocities[2] = sqrt(w2(1));
        _comm.angular_velocities[3] = sqrt(w2(0));
      }
      else {
        ROS_WARN("w problem");
        //cout<<controlInput<<endl;
        //cout<<_Ep<<endl;
      }
      //assert (w2(0)>=0 && w2(1)>=0 && w2(2)>=0 && w2(3)>=0);
    }
    else {
      _comm.header.stamp = ros::Time::now();
      bool landed = true;

      for (int i=0; i<4; i++) {
        _comm.angular_velocities[i] = _comm.angular_velocities[i]-1;
        if (_comm.angular_velocities[i]<0) {
           _comm.angular_velocities[i] = 0;
        }
        else landed = false;
      }

      if(landed) _landed = true;
    }

    _cmd_vel_pub.publish (_comm);
    r.sleep();
  }

}

void QUAD_CTRL::run() {
    boost::thread ctrl_loop_t ( &QUAD_CTRL::ctrl_loop, this);
}

bool shut=false;
void cb_server() {
  while (!shut)
    ros::spinOnce();
}

int main( int argc, char** argv) {

    ros::init(argc, argv, "quad_controller" );

    double limits[6]={-1,6,-1,6,0,5};

    std::vector<std::array<double, 4>> wayPoints;
    std::array<double, 4> point0 = {1,0,0.06,0};
    std::array<double, 4> point1 = {3,3,3,0};
    std::array<double, 4> point2 = {5.4,1.4,0.07,0};
    wayPoints.push_back(point2);
    wayPoints.push_back(point1);
    //wayPoints.push_back(point0);

    QUAD_CTRL c;
    QUAD_PLAN p(limits);

    ROS_INFO("Programma attivo");
    c.run();
    p.run();
    boost::thread cb_server_t ( &cb_server );

    while (wayPoints.size()>0) {
      std::array<double, 4> point = wayPoints.back();
      wayPoints.pop_back();
      double init[4];
      double goal[] = {point[0],point[1],point[2],point[3]};

      c.getPose(init);
      //cout<<endl<<init[0]<<" "<<init[1]<<" "<<init[2]<<" "<<init[3]<<endl;
      p.setGoal(init, goal);
      while(!p.isPlanned()) usleep(1000);

      c.setTraj(p.poses, p.velocities, p.accelerations);
      while(!c.isTrajCompleted()) usleep(1000);

      //TODO FLIP
      ROS_WARN("WAYPOINT REACHED!");
      sleep(1);
    }

    c.land();
    while (!c.isLanded()) usleep(1000);
    shut = true;
    ROS_INFO("PATH COMPLETED!");

    sleep(1);

    return 0;

}
