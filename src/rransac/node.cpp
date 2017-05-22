#include <ros/ros.h>
#include "rransac/rransac.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rransac_node");
  ros::NodeHandle nh;

	std::cout << "node started" << std::endl;

  // receive parameters from launchfile
  // bool x, y, z;
	// nh.param<bool>("show_x", x, false);
	// nh.param<bool>("show_y", y, false);
	// nh.param<bool>("show_z", z, false);
  // the original code used "nh_private_": "ros::NodeHandle nh_private_("~");"


  visual_mtt::RRANSAC rransac;
  // this was originally "visual_mtt::RRANSAC rransac()"
  // this was changed in order to create the object and call the constructor


  ros::spin();

  return 0;
}