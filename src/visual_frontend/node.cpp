#include <ros/ros.h>
#include "visual_frontend/visual_frontend.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "visual_frontend_node");

  // instantiate the visual_mtt::VisualFrontend class
  visual_mtt::VisualFrontend frontend;

  ros::spin();
  return 0;
}