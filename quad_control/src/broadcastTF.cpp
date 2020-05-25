#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

   tf::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStampedbase, transformStampedworld, transformStampedmap;


  transformStampedbase.header.frame_id = "hummingbird/base_link";
  transformStampedbase.child_frame_id = "hummingbird/base_linkNED";
  transformStampedbase.transform.translation.x = 0.0;
  transformStampedbase.transform.translation.y = 0.0;
  transformStampedbase.transform.translation.z = 0.0;
  tf::Quaternion q;
        q.setEulerZYX(M_PI/2, 0, M_PI);
  transformStampedbase.transform.rotation.x = q.x();
  transformStampedbase.transform.rotation.y = q.y();
  transformStampedbase.transform.rotation.z = q.z();
  transformStampedbase.transform.rotation.w = q.w();

  transformStampedworld.header.frame_id = "world";
  transformStampedworld.child_frame_id = "worldNED";
  transformStampedworld.transform.translation.x = 0.0;
  transformStampedworld.transform.translation.y = 0.0;
  transformStampedworld.transform.translation.z = 0.0;
  transformStampedworld.transform.rotation.x = q.x();
  transformStampedworld.transform.rotation.y = q.y();
  transformStampedworld.transform.rotation.z = q.z();
  transformStampedworld.transform.rotation.w = q.w();

  transformStampedmap.header.frame_id = "world";
  transformStampedmap.child_frame_id = "map";
  transformStampedmap.transform.translation.x = 0.0;
  transformStampedmap.transform.translation.y = 0.0;
  transformStampedmap.transform.translation.z = 0.0;
  transformStampedmap.transform.rotation.x = 0;
  transformStampedmap.transform.rotation.y = 0;
  transformStampedmap.transform.rotation.z = 0;
  transformStampedmap.transform.rotation.w = 1;

  ros::Rate rate(200.0);
  while (node.ok()){
    transformStampedbase.header.stamp = ros::Time::now();
    transformStampedworld.header.stamp = ros::Time::now();
    transformStampedmap.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStampedbase);
    tfb.sendTransform(transformStampedworld);
    tfb.sendTransform(transformStampedmap);
    rate.sleep();
    //printf("sending\n");
  }

};
