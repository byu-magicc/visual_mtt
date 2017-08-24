#include <ros/ros.h>
#include "camera_sim/camera_sim.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "camera_sim");

  // instantiate the CameraSim class
  camera_sim::CameraSim camera_sim;

  // read video file if applicable
  camera_sim.play_video();

  // prepare for rosbag callback if applicable
  ros::spin();
  return 0;
}