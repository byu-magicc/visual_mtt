#include <ros/ros.h>
#include "rransac/rransac.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "rransac_node");

  // instantiate the visual_mtt::RRANSAC shell class
  visual_mtt::RRANSAC rransac;

  ros::spin();
  return 0;
}