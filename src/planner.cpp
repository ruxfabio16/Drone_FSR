#include "../include/quad_control/planner.h"



//----------- PLANNER ----------

double QUAD_PLAN::nDist (const Node* n1, const Node* n2) {
  Vector3d dist = n2->p - n1->p;
  return dist.norm();
  }

Node* QUAD_PLAN::searchTree(Node* root, Node* newPose) {
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


bool QUAD_PLAN::AstarSearchTree(Node* root, Node* goal, bool rootTree) {
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


void QUAD_PLAN::setGoal(const double* init, const double * goal) {
  for (int i=0; i<4; i++) {
    _init_pose[i] = init[i];
    _goal_pose[i] = goal[i];
  }

  _new_goal = true;
}

QUAD_PLAN::QUAD_PLAN(const double * boundaries) {

	_path_pub = _nh.advertise<nav_msgs::Path>("path", 0);
  _filtered_path_pub = _nh.advertise<nav_msgs::Path>("filteredPath", 0);
  _poses_pub = _nh.advertise<geometry_msgs::Pose>("planned/Pose", 0);
  _vel_pub = _nh.advertise<geometry_msgs::Twist>("planned/Velocity", 0);
  _acc_pub = _nh.advertise<geometry_msgs::Accel>("planned/Acceleration", 0);
  //_traject_pub = _nh.advertise<nav_msgs::Path>("trajectory", 0);

	_Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.5, 0.5, 0.3));
	//fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree("/home/eugenio/arena.binvox.bt")));
	_tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
	//_octree_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &QUAD_PLAN::octomapCallback, this);

  for (int i=0; i<6; i++)
    _boundaries[i] = boundaries[i];

  _new_goal = false;
  _tresh = 0.02;
  _planned = false;
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
    //ROS_INFO("Test Collisione %f",i);
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
  filtered_path.header.frame_id = "map";


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
    ROS_INFO("Tries: %d",tries);
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
      //ROS_INFO("goalTree");
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

  size_t nPoses = generated_path.poses.size();
  double yawInc = (goal->yaw - root->yaw)/nPoses;
  for (int i=0; i<nPoses; i++) {
    tf::Quaternion quat;
    quat.setRPY(0,0,root->yaw + yawInc*i);
    generated_path.poses[i].pose.orientation.x = quat[0];
    generated_path.poses[i].pose.orientation.y = quat[1];
    generated_path.poses[i].pose.orientation.z = quat[2];
    generated_path.poses[i].pose.orientation.w = quat[3];

    filtered_path.poses.push_back(generated_path.poses[i]);
  }

  generateTraj();
  for (int i=0; i<100; i++)
    filterPath();

  while( ros::ok() ) {
    _planned = true;
    _path_pub.publish( generated_path );
    _filtered_path_pub.publish( filtered_path );
    r.sleep();
  }
}

void QUAD_PLAN::filterPath() {
  size_t nPoses = filtered_path.poses.size();
  for (int i=1; i<(nPoses-1); i++) {
    filtered_path.poses[i].pose.position.x = (filtered_path.poses[i-1].pose.position.x + filtered_path.poses[i].pose.position.x + filtered_path.poses[i+1].pose.position.x)/3.0;
    filtered_path.poses[i].pose.position.y = (filtered_path.poses[i-1].pose.position.y + filtered_path.poses[i].pose.position.y + filtered_path.poses[i+1].pose.position.y)/3.0;
    filtered_path.poses[i].pose.position.z = (filtered_path.poses[i-1].pose.position.z + filtered_path.poses[i].pose.position.z + filtered_path.poses[i+1].pose.position.z)/3.0;

  }
}

void QUAD_PLAN::generateTraj() {
  double stepTime = 0.001; // 1 ms
  double tf = 0.2;

  size_t nPoses = filtered_path.poses.size();
  for (int i=0; i<(nPoses-1); i++) {
    Vector3d pointi;
    Vector3d pointf;
    pointi(0) = filtered_path.poses[i].pose.position.x;
    pointi(1) = filtered_path.poses[i].pose.position.y;
    pointi(2) = filtered_path.poses[i].pose.position.z;

    pointf(0) = filtered_path.poses[i+1].pose.position.x;
    pointf(1) = filtered_path.poses[i+1].pose.position.y;
    pointf(2) = filtered_path.poses[i+1].pose.position.z;

    std::vector<Vector4d> aVec;
    for (int coord=0; coord<3; coord++) {
      double xi = pointi[coord];
      double xf = pointf[coord];
      double xip = 0;
      double xfp = 0;

      Vector4d a(0,0,0,0);
      Vector2d a23;
      Vector2d b;
      Matrix2d A;
      a(0) = xi;
      a(1) = xip;

      A(0,0) = tf*tf*tf; A(0,1) = tf*tf;
      A(1,0) = 3*tf*tf;  A(1,1) = 2*tf;
      b(0) = xf - a(0) - a(1)*tf;
      b(1) = xfp - a(1);

      a23 = A.inverse() * b;
      a(2) = a23(0);
      a(3) = a23(1);

      aVec.push_back(a);
    }

    for (double t=0; t<tf; t+=stepTime) {
      geometry_msgs::Pose pos;
      geometry_msgs::Twist vel;
      geometry_msgs::Accel acc;

      Vector4d a = aVec[0];
      pos.position.x = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0);
      vel.linear.x = 3*a(3)*t*t + 2*a(2)*t + a(1);
      acc.linear.x = 6*a(3)*t + 2*a(2);

      a = aVec[1];
      pos.position.y = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0);
      vel.linear.y = 3*a(3)*t*t + 2*a(2)*t + a(1);
      acc.linear.y = 6*a(3)*t + 2*a(2);

      a = aVec[2];
      pos.position.z = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0);
      vel.linear.z = 3*a(3)*t*t + 2*a(2)*t + a(1);
      acc.linear.z = 6*a(3)*t + 2*a(2);

      poses.push_back(pos);
      velocities.push_back(vel);
      accelerations.push_back(acc);

      _poses_pub.publish(pos);
      _vel_pub.publish(vel);
      _acc_pub.publish(acc);
    }

  }

}
