#include "visual_frontend/visual_frontend.h"

namespace visual_mtt {

VisualFrontend::VisualFrontend()
{
  // receive parameters from launchfile
  // bool x, y, z;
	// nh.param<bool>("show_x", x, false);
	// nh.param<bool>("show_y", y, false);
	// nh.param<bool>("show_z", z, false);
  // the original code used "nh_private_": "ros::NodeHandle nh_private_("~");"

  std::cout << "initialized VisualFrontend object inside node" << std::endl; // temporary

  // ROS stuff
  sub_video = nh.subscribe("video", 1, &VisualFrontend::callback_video, this);
  sub_imu = nh.subscribe("imu", 1, &VisualFrontend::callback_imu, this);
  sub_tracks = nh.subscribe("tracks", 1, &VisualFrontend::callback_tracks, this);
	pub = nh.advertise<std_msgs::Float32>("measurements_and_homographies", 1); // temporary dummy std_msgs for compilation

}


void VisualFrontend::callback_video(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
  // BIG QUESTION:
  // imu data and camera are supposed to be perfectly synchronized
  // does this mean that we'll expect "video" and "imu" messages to have
  // identical timestamps? or will this be one message?
  // ---------------------------------------------------



	// perform decimation logic based on params and decide whether
  // to keep frame or skip

  // resize frame to lower resolution

  // add (high and low res) frames to collections (class members)
    // a short history is needed for the low res for the sliding
    // a short history is needed for the high res so the track recognition can
    // locate the high-res frame associated with the track it's subscribing to

  // wait for homography calculation before executing measurement sources

}

void VisualFrontend::callback_imu(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
	// see BIG QUESTION above

  // another QUESTION:
  // if the frame and IMU don't come in together in the same message, will
  // we be getting the full IMU at 1000+ Hz for use in the homography filter?
  // NOTE: need better understanding of homography filter to answer this.

  // another QUESTION:
  // for in-frame tracking, we'll need 2 homographies:
  // one between the sliding frames for background subtraction
  // one between the two recent frames for updating history and estimates
  // if the homography filter operates on a manifold, maybe it can help?
  // NOTE: need better understanding of homography filter to answer this.
  // ---------------------------------------------------

  // add IMU measurements to class

  // call homography filter

  // execute measurement sources (publishing occurs at the end)



}

void VisualFrontend::callback_tracks(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
	// save most recent track information in class (for use in measurement
  // sources such as direct methods)

  // call track_recognition bank (will use newest information and the high-res
  // video associated with the most recent update to maintain id descriptors.)

}




}