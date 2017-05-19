#include <ros/ros.h>
#include "rransac/rransac.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rransac_node");
  ros::NodeHandle nh;

  visual_mtt::RRANSAC rransac(); 

  ros::spin();

  return 0;
}