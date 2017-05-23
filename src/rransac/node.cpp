#include <ros/ros.h>
#include "rransac/rransac.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "rransac_node");

  // echo node status (temporary for debugging)
	std::cout << "rransac_node started" << std::endl;

  // instantiate the visual_mtt::RRANSAC shell class
  visual_mtt::RRANSAC rransac;

  ros::spin();
  return 0;
}