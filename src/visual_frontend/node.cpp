#include <ros/ros.h>
#include "visual_frontend/visual_frontend.h"

#define BLUE "\033[1;34m"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "visual_frontend_node");

  // Grab the install path of OpenCV
  int s = cv::getBuildInformation().find("Install path:");
  int e = cv::getBuildInformation().find('\n', s);
  ROS_INFO(BLUE "OpenCV %s", cv::getBuildInformation().substr(s, e-s).c_str());
#if OPENCV_CUDA
  ROS_INFO(BLUE "Visual MTT CUDA enabled with %i device(s).", cv::cuda::getCudaEnabledDeviceCount());
#endif

  // instantiate the visual_frontend::VisualFrontend class
  visual_frontend::VisualFrontend frontend;

  ros::spin();
  return 0;
}