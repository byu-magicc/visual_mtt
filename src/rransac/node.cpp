#include <ros/ros.h>
#include "rransac/rransac.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "rransac_node");

  // instantiate the rransac::RRANSAC shell class
  rransac::RRANSAC rransac;

  ros::spin();
  return 0;
}