#include <ros/ros.h>
#include "visual_frontend/visual_frontend.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_frontend_node");
  ros::NodeHandle nh;

  std::cout << "visual_frontend_node started" << std::endl; // temporary debugging

  // receive parameters from launchfile
  // bool x, y, z;
	// nh.param<bool>("show_x", x, false);
	// nh.param<bool>("show_y", y, false);
	// nh.param<bool>("show_z", z, false);
  // the original code used "nh_private_": "ros::NodeHandle nh_private_("~");"

  visual_mtt::VisualFrontend frontend;

  ros::spin();

  return 0;
}