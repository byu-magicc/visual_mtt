#include "rransac/rransac.h"

namespace visual_mtt {

RRANSAC::RRANSAC()
{
  // receive parameters from launchfile
  // bool x, y, z; // move to header
  // nh.param<bool>("show_x", x, false);
  // nh.param<bool>("show_y", y, false);
  // nh.param<bool>("show_z", z, false);
  // the original code used "nh_private_": "ros::NodeHandle nh_private_("~");"

  // instantiate the rransac::Tracker library class
  tracker_ = rransac::Tracker(params_);
  std::cout << "initialized rransac object inside node" << std::endl; // temporary

  // ROS stuff
  // TODO: fix terrible topic name
  sub = nh.subscribe("measurements_and_homographies", 1, &RRANSAC::callback, this); // noob question: is there a way to specify message type here? (in addition to specifying in the callback function)
  pub = nh.advertise<std_msgs::Float32>("tracks", 1); // temporary dummy std_msgs for compilation
}

void RRANSAC::callback(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
  // homographies and measurements arrive in one message synchronized
  // for the current "scan"

  // separate homography from measurements

  // apply measurements to tracker_

  // apply homographies to tracker_

  // retrieve good models and publish
  // the message will have a timestamp associated with the frame of this
  // scan AND the current time (timestamp of model updates)
}



}