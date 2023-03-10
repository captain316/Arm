#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  
  tf::TransformListener listener;

  ros::Rate rate(30.0);
  geometry_msgs::Pose target_pose; 
  while (node.ok()){
      tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("base_link", "fingers_frame", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("base_link", "fingers_frame",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    poseTFToMsg(transform, target_pose);
    cout << target_pose.position.x << endl;
    cout << target_pose.position.y << endl;
    cout << target_pose.position.z << endl;
    cout << "---------------------------------" << endl;
    

    rate.sleep();
  }
  return 0;
};