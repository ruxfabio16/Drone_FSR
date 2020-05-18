#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include "mav_msgs/Actuators.h"
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>

#include <random>
#include <iostream>
#include <ctime>



using namespace std;
using namespace Eigen;

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


//----------- PLANNER ----------


struct Node {
  std::vector<Node*> children;
  Node* parent;

  Vector3d p;
  double yaw;
};

double nDist (const Node* n1, const Node* n2) {
  Vector3d dist = n2->p - n1->p;
  return dist.norm();
  //return sqrt( pow(n1->p[0] - n2->p[0] , 2) + pow(n1->p[1] - n2->p[1] , 2) + pow(n1->p[2] - n2->p[2] , 2)  );
}

Node* searchTree(Node* root, Node* newPose) {
  Node * nearest = root;
  double bestDist = nDist(root, newPose);

  std::vector<Node*> open;
  open.push_back(root);

  while ( !(open.empty()) ) {
    Node * n = open.front();
    open.erase(open.begin());

    double dist = nDist(n, newPose);
    if (dist<bestDist) {
      bestDist = dist;
      nearest = n;
    }

    size_t size = n->children.size();

    for (int i=0; i<size; i++) {
      open.push_back( n->children[i] );
    }
  }

  return nearest;
}

nav_msgs::Path generated_path;

bool AstarSearchTree(Node* root, Node* goal, bool rootTree) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";

  std::vector<Node*> open;
  open.push_back(root);
  root->parent = root;
  double bestDist = nDist(root, goal)+100;
  Node * bestNode = root;
  bool found = false;

  while ( !(open.empty()) && !found) {
    size_t i=0;
    size_t bestPos = 0;
    for (i=0; i<open.size(); i++) {
      if ( (nDist(open[i], root) + nDist(open[i], goal)) < bestDist ) {
        bestNode = open[i]; //Trovo il migliore
        //ROS_INFO("Migliore trovato");
        bestPos=i;
      }
    }
    //ROS_INFO("Cancello nodo");
    open.erase(open.begin()+bestPos); //Lo elimino dai nodi da controllare
    //ROS_INFO("Nodo cancellato");
    if (bestNode==goal)
      found = true;
    else {
      size_t size = bestNode->children.size();

      for (int i=0; i<size; i++) {
        open.push_back( bestNode->children[i] );
        bestNode->children[i]->parent = bestNode;
      }
    }

  }

  if (found) {
    ROS_INFO("Astar trovato");
    Node * start = goal;
    while (start != root) {
      p.pose.position.x = start->p[0];
      p.pose.position.y = start->p[1];
      p.pose.position.z = start->p[2];

      p.pose.orientation.x = 0;
      p.pose.orientation.y = 0;
      p.pose.orientation.z = 0;
      p.pose.orientation.w = 1;

      if (rootTree)
        generated_path.poses.insert(generated_path.poses.begin(), p);
      else
        generated_path.poses.insert(generated_path.poses.end(), p);

      start = start->parent;
    }

    p.pose.position.x = root->p[0];
    p.pose.position.y = root->p[1];
    p.pose.position.z = root->p[2];
    if (rootTree)
      generated_path.poses.insert(generated_path.poses.begin(), p);
    else
      generated_path.poses.insert(generated_path.poses.end(), p);

  }

  return found;
}

class QUAD_PLAN {
    public:
        QUAD_PLAN(const double * boundaries);
        void run();
        void plan();
        bool isStateValid(const Node*, const Node*);
        void setGoal(const double* init, const double * goal);
    private:
        ros::NodeHandle _nh;

        ros::Publisher _path_pub;
        std::shared_ptr<fcl::CollisionGeometry> _Robot;
        std::shared_ptr<fcl::CollisionGeometry> _tree_obj;
    		ros::Subscriber _octree_sub;
        double _init_pose[4];
        double _goal_pose[4];
        double _boundaries[6];
        bool _new_goal;
        double _tresh;
};

void QUAD_PLAN::setGoal(const double* init, const double * goal) {
  for (int i=0; i<4; i++) {
    _init_pose[i] = init[i];
    _goal_pose[i] = goal[i];
  }

  _new_goal = true;
}

QUAD_PLAN::QUAD_PLAN(const double * boundaries) {

	_path_pub = _nh.advertise<nav_msgs::Path>("path", 0);

	_Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.5, 0.5, 0.3));
	//fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree("/home/eugenio/arena.binvox.bt")));
	_tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
	//_octree_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &QUAD_PLAN::octomapCallback, this);

  for (int i=0; i<6; i++)
    _boundaries[i] = boundaries[i];

  _new_goal = false;
  _tresh = 0.05;
}

bool QUAD_PLAN::isStateValid(const Node* q1, const Node* q2)
{
	fcl::CollisionObject treeObj((_tree_obj));
	fcl::CollisionObject robotObject(_Robot);

  Vector3d direction = q2->p - q1->p;
  double norma = direction.norm();
  direction = direction / direction.norm();
  bool collision = false;

  for (double i=0; i<=norma; i+=(norma/5) ) {
    ROS_INFO("Test Collisione %f",i);
  	// check validity of state defined by pos & rot
  	fcl::Vec3f translation(q1->p[0] + i*direction[0],q1->p[1] + i*direction[1], q1->p[2] + i*direction[2]);
  	fcl::Quaternion3f rotation(0, 0, 0, 1);
    //fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
  	robotObject.setTransform(rotation, translation);
  	fcl::CollisionRequest requestType(1,false,1,false);
  	fcl::CollisionResult collisionResult;
  	fcl::collide(&robotObject, &treeObj, requestType, collisionResult);
    if (!collision)
      collision = collisionResult.isCollision();
  }

	return(!collision);

}

void QUAD_PLAN::run() {
	boost::thread plan_t( &QUAD_PLAN::plan, this );
}

void QUAD_PLAN::plan() {
  ROS_INFO("Planner attivo");

  bool found = false;
  bool rootFound = false;
  bool goalFound = false;
  int tries = 0;

  ros::Rate r(10);

  while (!_new_goal) usleep(1);

  Node* goal = new Node;
  goal->p[0] = _goal_pose[0];
  goal->p[1] = _goal_pose[1];
  goal->p[2] = _goal_pose[2];
  goal->yaw = _goal_pose[3];

  Node* root = new Node;
  root->p[0] = _init_pose[0];
  root->p[1] = _init_pose[1];
  root->p[2] = _init_pose[2];
  root->yaw = _init_pose[3];

  generated_path.header.frame_id = "map";


  double mindistGoal = 1000;

  std::default_random_engine re;
  re.seed(std::random_device{}());
  std::uniform_int_distribution<int> pickGoal(0, 100);

  Node * q_rand_goal;
  Node * q_rand;
  Node* q_near;

  bool faseUnione = false;

  while(!found && tries<1500) {
    bool validState=false;
    double dist=0;
    tries++;
    ROS_INFO("Tries: %d - %d",tries,pickGoal(re));
    if (tries>1000) faseUnione=true;
    //Generate InitialPos tree
    q_rand = new Node;

    do{
      if (pickGoal(re)>=80)
        q_rand->p = goal->p;
      else {
        for (int i=0; i<3; i++) {
          std::uniform_int_distribution<int> uni(1000*_boundaries[2*i], 1000*_boundaries[2*i+1]);
          q_rand->p[i] = (double)(uni(re))/1000.0;
        }
      }
      q_near = searchTree(root, q_rand);
      dist = nDist(q_near, q_rand);

      if (dist>_tresh) {
        Vector3d direction = q_rand->p - q_near->p;
        direction = direction / direction.norm();
        q_rand->p = q_near->p + _tresh*direction;
      }

      if (isStateValid(q_near,q_rand)) {
        q_near->children.push_back(q_rand);
        if (q_rand->p == goal->p)
          rootFound = true;
        validState = true;
      }
      else
        ROS_INFO("Non valido");

    } while (!validState);

    if(rootFound) break;

    //Generate Goal Tree
    q_rand_goal = new Node;
    validState = false;

    do {
      ROS_INFO("goalTree");
      if (pickGoal(re)>=80)
        q_rand_goal->p = root->p;
      else {
        for (int i=0; i<3; i++) {
          std::uniform_int_distribution<int> uni(1000*_boundaries[2*i], 1000*_boundaries[2*i+1]);
          q_rand_goal->p[i] = (double)(uni(re))/1000.0;
        }
      }

      //ROS_INFO("Cercando");
      q_near = searchTree(goal, q_rand_goal);
      //ROS_INFO("trovato");
      dist = nDist(q_near, q_rand_goal);
      if (dist>_tresh) {
        Vector3d direction = q_rand_goal->p - q_near->p;
        direction = direction / direction.norm();
        q_rand_goal->p = q_near->p + _tresh*direction;
      }

      if (isStateValid(q_near,q_rand_goal)) {
        q_near->children.push_back(q_rand_goal);
        if (q_rand_goal->p == goal->p)
          goalFound = true;
        validState=true;
      }
      else
        ROS_INFO("Non valido");
      } while(!validState);

    if(goalFound) break;


    //Connect trees
    if (faseUnione) {
      if (tries==1001) ROS_INFO("Fase unione");

      //Connect q_rand_goal to Initial tree

        q_near = searchTree(root, q_rand_goal);

        dist = nDist(q_near, q_rand_goal);
        if (dist>_tresh) {
          Vector3d direction = q_rand_goal->p - q_near->p;
          direction = direction / direction.norm();
          q_rand_goal->p = q_rand_goal->p + _tresh*direction;
          if (isStateValid(q_near,q_rand_goal)) {
            q_near->children.push_back(q_rand_goal);
            validState = true;
          }
          else
            ROS_INFO("Non valido");
        }
        else  found = true;

      //Connect q_rand to Goal tree

      if (!found) {

          q_near = searchTree(goal, q_rand);
          dist = nDist(q_near, q_rand);
          if (dist>_tresh) {
            Vector3d direction = q_rand->p - q_near->p;
            direction = direction / direction.norm();
            q_rand->p = q_near->p + _tresh*direction;
            if (isStateValid(q_near,q_rand_goal)) {
              q_near->children.push_back(q_rand);
              validState = true;
            }
            else
              ROS_INFO("Non valido");
          }
          else  found = true;

      }


    }

  }

  if (rootFound) {
    if ( AstarSearchTree(root, q_near, true) ) ROS_INFO("Path trovato");
    else ROS_INFO("Path non trovato");
  }
  else if (goalFound) {
    if ( AstarSearchTree(goal, q_rand_goal, false) ) ROS_INFO("Path trovato");
    else ROS_INFO("Path non trovato");
  }
  else if(found) {
    ROS_INFO("Connessione trovata");
    if ( AstarSearchTree(root, q_near, true) && AstarSearchTree(goal, q_rand_goal, false) ) ROS_INFO("Path trovato");
    else ROS_INFO("Path non trovato");
  }
  else ROS_INFO("Connessione non trovata");

  while( ros::ok() ) {
    _path_pub.publish( generated_path );
    r.sleep();
  }
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
    double goal[] = {3,3,3,0};
    p.setGoal(init, goal);

    ros::spin();


    return 0;

}
