#include "../include/quad_control/planner.h"



//----------- PLANNER ----------

double QUAD_PLAN::nDist (const Node* n1, const Node* n2) {
  Vector3d dist = n2->p - n1->p;
  return dist.norm();
  }

Node* QUAD_PLAN::searchTree(Node* root, Node* newPose) {
  //ROS_INFO("Comincio ricerca");
  Node * nearest = root;
  double bestDist = nDist(root, newPose);

  std::vector<Node*> open;
  open.push_back(root);

  int iter = 0;

  while ( !(open.empty()) && iter<2000) {
    iter++;
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

  //ROS_INFO("Finita ricerca");
  return nearest;
}

void QUAD_PLAN::clearTree(Node* root) {
  std::vector<Node*> open;
  open.push_back(root);

  while ( !(open.empty()) ) {
    Node * n = open.front();
    open.erase(open.begin());

    size_t size = n->children.size();

    for (int i=0; i<size; i++) {
      open.push_back( n->children[i] );
    }
    delete n;
  }

}


bool QUAD_PLAN::AstarSearchTree(Node* root, Node* goal, bool rootTree) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";

  if (!rootTree) goal = searchTree(root,goal);
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

    if (nDist(bestNode,goal)==0)
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
    if(rootTree)  ROS_INFO("Astar root trovato");
    else ROS_INFO("Astar goal trovato");

    Node * start = goal;
    while (start != root) {
      //ROS_INFO("Aggiungo");
    //while (nDist(start,root)!=0) {
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
//ROS_INFO("Finito");
  return found;
}


void QUAD_PLAN::setGoal(const double* init, const double * goal, const bool flip) {
  for (int i=0; i<4; i++) {
    _init_pose[i] = init[i];
    _goal_pose[i] = goal[i];
  }
  _flip = flip;

  _planned = false;

  usleep(1000);

  poses.clear();
  poses.resize(0);
  velocities.clear();
  velocities.resize(0);
  accelerations.clear();
  accelerations.resize(0);
  generated_path.poses.clear();
  filtered_path.poses.clear();
  cubic_path.poses.clear();

  _new_goal = true;
}

QUAD_PLAN::QUAD_PLAN(const double * boundaries) {

	_path_pub = _nh.advertise<nav_msgs::Path>("path", 0);
  _filtered_path_pub = _nh.advertise<nav_msgs::Path>("filteredPath", 0);
  _cubic_path_pub = _nh.advertise<nav_msgs::Path>("cubicPath", 0);
  _poses_pub = _nh.advertise<geometry_msgs::PoseStamped>("planned/Pose", 0);
  _vel_pub = _nh.advertise<geometry_msgs::TwistStamped>("planned/Velocity", 0);
  _acc_pub = _nh.advertise<geometry_msgs::AccelStamped>("planned/Acceleration", 0);
  //_traject_pub = _nh.advertise<nav_msgs::Path>("trajectory", 0);

	_Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Cylinder(0.3, 0.3));
	//fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
  std::string arena_path;
  _nh.getParam("arena_path", arena_path);
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(arena_path)));
	_tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
	//_octree_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &QUAD_PLAN::octomapCallback, this);


  for (int i=0; i<6; i++)
    _boundaries[i] = boundaries[i];

  _new_goal = false;
  _tresh = 0.02;
  _planned = false;
  _shutdown = false;
}

bool QUAD_PLAN::isStateValid(const Node* q1, const Node* q2) {
	fcl::CollisionObject treeObj((_tree_obj));
	fcl::CollisionObject robotObject(_Robot);

  Vector3d direction = q2->p - q1->p;
  double norma = direction.norm();
  direction = direction / direction.norm();
  bool collision = false;

  if (norma<=0) {
    //ROS_INFO("Norma");
    return false;
  }

  for (double i=0; i<=norma; i+=(norma/5) ) {
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
  boost::thread debug_t ( &QUAD_PLAN::debug_loop, this);
}

void QUAD_PLAN::plan() {
  ROS_INFO("Planner attivo");
  ros::Rate r(10);

  while (ros::ok()) {

    while (!_new_goal) usleep(1000);

    if(!_flip) {

      bool found = false;
      bool rootFound = false;
      bool goalFound = false;
      int tries = 0;

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
      cubic_path.header.frame_id = "map";


      double mindistGoal = 1000;

      std::default_random_engine re;
      re.seed(std::random_device{}());
      std::uniform_int_distribution<int> pickGoal(0, 100);

      Node * q_rand_goal;
      Node * q_rand;
      Node* q_near;

      bool faseUnione = false;

      while(!found && tries<1500) {
        usleep(1000);
        bool validState=false;
        double dist=0;
        tries++;
        //ROS_INFO("Tries: %d",tries);
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
          //else
          //  ROS_INFO("Non valido");

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
          //else
          //  ROS_INFO("Non valido");
          } while(!validState);

        if(goalFound) {
          //ROS_INFO("Trovato goal: break");
          break;
        }

        validState=false;
        //Connect trees
        if (faseUnione) {
          if (tries==1001) ROS_INFO("Fase unione");

          //Connect q_rand_goal to Initial tree
            double appo[3];
            for (int i=0; i<3; i++)
              appo[i] = q_rand_goal->p[i];
            q_rand_goal = new Node;
            for (int i=0; i<3; i++)
              q_rand_goal->p[i] = appo[i];

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
              //else
                //ROS_INFO("Non valido");
            }
            else  found = true;

            validState=false;
          //Connect q_rand to Goal tree

          if (!found && validState) {

              for (int i=0; i<3; i++)
                appo[i] = q_rand->p[i];
              q_rand = new Node;
              for (int i=0; i<3; i++)
                q_rand->p[i] = appo[i];

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
                //else
                  //ROS_INFO("Non valido");
              }
              else  found = true;

          }


        }

      }

      ROS_INFO("Cerco un path...");

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
        if ( AstarSearchTree(root, q_near, true) && AstarSearchTree(goal, q_near, false) ) ROS_INFO("Path trovato");
        else ROS_INFO("Path non trovato");
      }
      else ROS_INFO("Connessione non trovata");

      ROS_INFO("Iterazioni: %d",tries);

      size_t nPoses = generated_path.poses.size();
      for (int i=0; i<nPoses; i++) {
        filtered_path.poses.push_back(generated_path.poses[i]);
      }

      for (int i=0; i<100; i++)
        filterPath();

      generateTraj();

      clearTree(root);
      if(!found) clearTree(goal);

      _new_goal = false;
      _planned = true;
      while( !_shutdown && !_new_goal ) {
        _path_pub.publish( generated_path );
        _filtered_path_pub.publish( filtered_path );
        _cubic_path_pub.publish( cubic_path );
        r.sleep();
      }

    }
    else {
      planFlip();
      _new_goal = false;
      _planned = true;
    }

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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void QUAD_PLAN::generateTraj() {
  double stepTime = 1.0/TimeRate; // 1 ms
  double tf = 0.1;
  double absolT=0;
  uint32_t seconds = 0;
  uint32_t nsec = 0;


  size_t nPoses = filtered_path.poses.size();
  for (int i=0; i<(nPoses-1); i++) {
    Vector3d pointi;
    Vector3d pointf;
    Vector3d pointf1;
    pointi(0) = filtered_path.poses[i].pose.position.x;
    pointi(1) = filtered_path.poses[i].pose.position.y;
    pointi(2) = filtered_path.poses[i].pose.position.z;

    pointf(0) = filtered_path.poses[i+1].pose.position.x;
    pointf(1) = filtered_path.poses[i+1].pose.position.y;
    pointf(2) = filtered_path.poses[i+1].pose.position.z;

    if (i<(nPoses-2)) {
      pointf1(0) = filtered_path.poses[i+2].pose.position.x;
      pointf1(1) = filtered_path.poses[i+2].pose.position.y;
      pointf1(2) = filtered_path.poses[i+2].pose.position.z;
    }

    Vector3d dist = pointf-pointi;
    tf = dist.norm()/0.2;

    std::vector<Vector4d> aVec;
    Vector3d xp_old;
    for (int coord=0; coord<3; coord++) {
      double xi = pointi[coord];
      double xf = pointf[coord];
      double xf1 = pointf1[coord];
      double xip = 0;
      double xfp = 0;


      double vk = (xf-xi)/tf;
      double vk1 = (xf1-xf)/tf;

      if (i==0) {
        xip = 0;
        if (sgn(vk)!=sgn(vk1))
          xfp = 0;
        else
          xfp = (vk+vk1)/2;
      }
      else if (i<(nPoses-2)) {
        xip = xp_old[coord];
        if (sgn(vk)!=sgn(vk1))
          xfp = 0;
        else
          xfp = (vk+vk1)/2;
      }
      else {
        xip = xp_old[coord];
        xfp = 0;
      }

      xp_old[coord] = xfp;

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
      a(2) = a23(1);
      a(3) = a23(0);

      aVec.push_back(a);
    }

    for (double t=0; t<=tf; t+=stepTime) {
      geometry_msgs::PoseStamped pos;
      geometry_msgs::TwistStamped vel;
      geometry_msgs::AccelStamped acc;

      absolT += stepTime;
      nsec += stepTime*1000000000;
      if (nsec>1000000000) {
        nsec = nsec - 1000000000;
        seconds ++;
      }

      ros::Time clock(seconds,nsec);
      clock == ros::Time::now();

      pos.header.frame_id="map";
      vel.header.frame_id="map";
      acc.header.frame_id="map";
      pos.header.stamp=clock;
      vel.header.stamp=clock;
      acc.header.stamp=clock;

      Vector4d a = aVec[0];
      pos.pose.position.x = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0);
      vel.twist.linear.x = 3*a(3)*t*t + 2*a(2)*t + a(1);
      acc.accel.linear.x = 6*a(3)*t + 2*a(2);

      a = aVec[1];
      pos.pose.position.y = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0);
      vel.twist.linear.y = 3*a(3)*t*t + 2*a(2)*t + a(1);
      acc.accel.linear.y = 6*a(3)*t + 2*a(2);

      a = aVec[2];
      pos.pose.position.z = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0);
      vel.twist.linear.z = 3*a(3)*t*t + 2*a(2)*t + a(1);
      acc.accel.linear.z = 6*a(3)*t + 2*a(2);

      poses.push_back(pos);
      velocities.push_back(vel);
      accelerations.push_back(acc);

    }

  }

  nPoses = poses.size();
  tf=absolT;
  ROS_INFO("Durata traiettoria: %f s",tf);
  double t=0;
  double xi = _init_pose[3];
  double xf = _goal_pose[3];
  double xip = 0;
  double xfp = 0;

  tf::Matrix3x3 Ri_tf, Rf_tf;
  Matrix3d Ri,Rf;
  Ri_tf.setRPY(0,0,xi);
  Rf_tf.setRPY(0,0,xf);
  tf::matrixTFToEigen(Ri_tf,Ri);
  tf::matrixTFToEigen(Rf_tf,Rf);

  Matrix3d Rif = Ri.transpose()*Rf;

  xi = 0;
  xf = acos( 0.5*(Rif(0,0)+Rif(1,1)+Rif(2,2)-1) );
  Vector3d ri;
  if (xf!=0 && xf!=M_PI) {
    ri << Rif(2,1)-Rif(1,2) , Rif(0,2)-Rif(2,0) , Rif(1,0)-Rif(0,1) ;
    ri = ri/(2*sin(xf));
  }
  else if (xf == 0) ri << 0,0,0;
  else if (xf == M_PI) {
    ri << Rif(0,0)+1 , Rif(1,1)+1 , Rif(2,2)+1 ;
    ri = ri/2;
    ri(0) = sqrt(ri(0));
    ri(1) = sqrt(ri(1));
    ri(2) = sqrt(ri(2));
  }

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
  a(2) = a23(1);
  a(3) = a23(0);

  double theta, thetad, thetadd;
  Vector3d wi, wid;

  for (int i=0; i<nPoses; i++) {

    theta = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0) ;
    thetad = 3*a(3)*t*t + 2*a(2)*t + a(1) ;
    thetadd = 6*a(3)*t + 2*a(2) ;

    Matrix3d R_i;
    R_axisAngle(theta,ri, R_i);
    //cout<<R_i.determinant()<<endl;
    wi = thetad*ri;
    wid = thetadd*ri;

    Matrix3d Rb_des = Ri*R_i;
    Vector3d wbb_des = Rb_des.transpose()*Ri*wi;
    Vector3d wbbd_des = Rb_des.transpose()*Ri*wid;
    tf::Matrix3x3 Rb_des_tf;
    tf::matrixEigenToTF(Rb_des, Rb_des_tf);

    tf::Quaternion quat;
    Rb_des_tf.getRotation(quat);
    poses[i].pose.orientation.x = quat[0];
    poses[i].pose.orientation.y = quat[1];
    poses[i].pose.orientation.z = quat[2];
    poses[i].pose.orientation.w = quat[3];

    velocities[i].twist.angular.x = wbb_des(0);
    velocities[i].twist.angular.y = wbb_des(1);
    velocities[i].twist.angular.z = wbb_des(2);

    accelerations[i].accel.angular.x = wbbd_des(0);
    accelerations[i].accel.angular.y = wbbd_des(1);
    accelerations[i].accel.angular.z = wbbd_des(2);

    t+=stepTime;
  }


  geometry_msgs::PoseStamped pos;
  geometry_msgs::TwistStamped vel;
  geometry_msgs::AccelStamped acc;
  pos = poses.back();
  for (int i=0; i<5; i++) {
    poses.push_back(pos);
    velocities.push_back(vel);
    accelerations.push_back(acc);
  }

  cubic_path.poses = poses;

}

void QUAD_PLAN::planFlip() {
  double stepTime = 1.0/TimeRate; // 1 ms
  double tf = 0.3;
  uint32_t seconds = 0;
  uint32_t nsec = 0;

  ROS_INFO("Durata traiettoria: %f s",tf);


  double xi = 0;
  double xf = M_PI;
  double xip = 0;
  double xfp = 3*M_PI/tf;

  for (int j=0; j<2; j++) {

    if (j>0) {
      xi = M_PI;
      xf = 2*M_PI;
      xip = xfp;
      xfp = 0;
    }

    tf::Matrix3x3 Ri_tf, Rf_tf;
    Matrix3d Ri,Rf;
    Ri_tf.setRPY(xi,0,_init_pose[3]);
    Rf_tf.setRPY(xf,0,_init_pose[3]);
    tf::matrixTFToEigen(Ri_tf,Ri);
    tf::matrixTFToEigen(Rf_tf,Rf);

    Matrix3d Rif = Ri.transpose()*Rf;

    xi = 0;
    xf = acos( 0.5*(Rif(0,0)+Rif(1,1)+Rif(2,2)-1) );
    Vector3d ri;
    if (xf!=0 && xf!=M_PI) {
      ri << Rif(2,1)-Rif(1,2) , Rif(0,2)-Rif(2,0) , Rif(1,0)-Rif(0,1) ;
      ri = ri/(2*sin(xf));
    }
    else if (xf == 0) ri << 0,0,0;
    else if (xf == M_PI) {
      ri << Rif(0,0)+1 , Rif(1,1)+1 , Rif(2,2)+1 ;
      ri = ri/2;
      ri(0) = sqrt(ri(0));
      ri(1) = sqrt(ri(1));
      ri(2) = sqrt(ri(2));
    }

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
    a(2) = a23(1);
    a(3) = a23(0);

    double theta, thetad, thetadd;
    Vector3d wi, wid;

    for (double t=0; t<=tf; t+=stepTime) {

      geometry_msgs::PoseStamped pos;
      geometry_msgs::TwistStamped vel;
      geometry_msgs::AccelStamped acc;

      ros::Time clock = ros::Time::now();

      pos.header.frame_id="map";
      vel.header.frame_id="map";
      acc.header.frame_id="map";
      pos.header.stamp=clock;
      vel.header.stamp=clock;
      acc.header.stamp=clock;

      pos.pose.position.x = _init_pose[0];
      pos.pose.position.y = _init_pose[1];
      pos.pose.position.z = _init_pose[2]+1.5;

      theta = a(3)*t*t*t + a(2)*t*t + a(1)*t + a(0) ;
      thetad = 3*a(3)*t*t + 2*a(2)*t + a(1) ;
      thetadd = 6*a(3)*t + 2*a(2) ;

      Matrix3d R_i;
      R_axisAngle(theta,ri, R_i);
      //cout<<R_i.determinant()<<endl;
      wi = thetad*ri;
      wid = thetadd*ri;

      Matrix3d Rb_des = Ri*R_i;
      Vector3d wbb_des = Rb_des.transpose()*Ri*wi;
      Vector3d wbbd_des = Rb_des.transpose()*Ri*wid;
      tf::Matrix3x3 Rb_des_tf;
      tf::matrixEigenToTF(Rb_des, Rb_des_tf);

      tf::Quaternion quat;
      Rb_des_tf.getRotation(quat);
      pos.pose.orientation.x = quat[0];
      pos.pose.orientation.y = quat[1];
      pos.pose.orientation.z = quat[2];
      pos.pose.orientation.w = quat[3];

      vel.twist.angular.x = wbb_des(0);
      vel.twist.angular.y = wbb_des(1);
      vel.twist.angular.z = wbb_des(2);

      acc.accel.angular.x = wbbd_des(0);
      acc.accel.angular.y = wbbd_des(1);
      acc.accel.angular.z = wbbd_des(2);

      poses.push_back(pos);
      velocities.push_back(vel);
      accelerations.push_back(acc);

    }
  }


  geometry_msgs::PoseStamped pos;
  geometry_msgs::TwistStamped vel;
  geometry_msgs::AccelStamped acc;
  pos = poses.back();
  pos.pose.position.z = _init_pose[2];

  for (int i=0; i<5; i++) {
    poses.push_back(pos);
    velocities.push_back(vel);
    accelerations.push_back(acc);
  }

  cubic_path.poses = poses;
  filtered_path.poses = poses;
  generated_path.poses = poses;

}

void QUAD_PLAN::debug_loop() {
  ros::Rate r(TimeRate);

  while(!_planned) usleep(100);
  ROS_INFO("Publishing trajectory");

  size_t size = poses.size();

  while (!_shutdown) {
    for(int i=0; (i<size && !_shutdown && _planned) ; i++) {
      _poses_pub.publish(poses[i]);
      _vel_pub.publish(velocities[i]);
      _acc_pub.publish(accelerations[i]);
      r.sleep();
    }
  }

}

void QUAD_PLAN::R_axisAngle(double th, Vector3d r, Matrix3d &R){
  double cc = 1-cos(th);

  R << r(0)*r(0)*cc+cos(th)       , r(0)*r(1)*cc-r(2)*sin(th) , r(0)*r(2)*cc+r(1)*sin(th),
       r(0)*r(1)*cc+r(2)*sin(th)  , r(1)*r(1)*cc+cos(th)      , r(1)*r(2)*cc-r(0)*sin(th),
       r(0)*r(2)*cc-r(1)*sin(th)  , r(1)*r(2)*cc+r(0)*sin(th) , r(2)*r(2)*cc+cos(th);

}
