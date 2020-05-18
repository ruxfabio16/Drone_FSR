#include "ros/ros.h"
#include "boost/thread.hpp"
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

#include <random>
#include <iostream>
#include <ctime>

using namespace std;
using namespace Eigen;

struct Node {
  std::vector<Node*> children;
  Node* parent;

  Vector3d p;
  double yaw;
};

class QUAD_PLAN {
    public:
        QUAD_PLAN(const double * boundaries);
        void run();
        void plan();
        void setGoal(const double* init, const double * goal);
        nav_msgs::Path generated_path;
        nav_msgs::Path filtered_path;
        std::vector<geometry_msgs::Pose> poses;
        std::vector<geometry_msgs::Twist> velocities;
        std::vector<geometry_msgs::Accel> accelerations;
        bool isPlanned() {return _planned;};
    private:
        ros::NodeHandle _nh;

        ros::Publisher _path_pub;
        ros::Publisher _filtered_path_pub;
        ros::Publisher _poses_pub;
        ros::Publisher _vel_pub;
        ros::Publisher _acc_pub;
        std::shared_ptr<fcl::CollisionGeometry> _Robot;
        std::shared_ptr<fcl::CollisionGeometry> _tree_obj;
    		ros::Subscriber _octree_sub;
        double _init_pose[4];
        double _goal_pose[4];
        double _boundaries[6];
        bool _new_goal;
        double _tresh;
        bool _planned;
        double nDist (const Node* n1, const Node* n2);
        Node* searchTree(Node* root, Node* newPose);
        bool AstarSearchTree(Node* root, Node* goal, bool rootTree);
        bool isStateValid(const Node*, const Node*);
        void generateTraj();
        void filterPath();
};
