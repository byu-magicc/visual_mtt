#include <ros/ros.h>
#include "visual_frontend/visual_frontend.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "visual_frontend_node");

  // instantiate the visual_frontend::VisualFrontend class
  visual_frontend::VisualFrontend frontend;

  ros::spin();
  return 0;
}