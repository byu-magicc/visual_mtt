#include <ros/ros.h>
#include "visual_frontend/visual_frontend.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "visual_frontend_node");

  std::cout << cv::getBuildInformation() << std::endl;
  std::cout << cudaDevices = cv::cuda::getCudaEnabledDeviceCount() << " CUDA-enabled devices" << std::endl;

  // instantiate the visual_frontend::VisualFrontend class
  visual_frontend::VisualFrontend frontend;

  ros::spin();
  return 0;
}