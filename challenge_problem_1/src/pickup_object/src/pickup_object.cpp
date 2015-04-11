#include <ros/ros.h>

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "pickup_object");

  ros::NodeHandle nh;

  //sleep for a moment while everything starts up
  ros::Duration(1.0).sleep();

  // your code goes here:



  ROS_INFO("Finished Picking up the cup!");

  return 0;
}
