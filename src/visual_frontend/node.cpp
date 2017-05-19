#include <ros/ros.h>
#include "visual_frontend/visual_frontend.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_frontend_node");
  ros::NodeHandle nh;

  visual_mtt::VisualFrontend frontend(); 

  ros::spin();

  return 0;
}