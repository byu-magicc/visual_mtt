#include "rransac/rransac.h"

namespace visual_mtt {

RRANSAC::RRANSAC()
{
  // instantiate the rransac::Tracker library class
  tracker_ = rransac::Tracker(params_);

  // ROS stuff
  sub = nh.subscribe("measurements", 1, &RRANSAC::callback, this);
  pub = nh.advertise<std_msgs::Float32>("tracks", 1);
}

// ----------------------------------------------------------------------------

void RRANSAC::callback(const std_msgs::Float32 data)
{
  // homographies and measurements arrive in one message synchronized
  // for the current "scan"

  // separate homography from measurements

  // apply measurements to tracker_

  // apply homographies to tracker_

  // retrieve good models and publish
  // the message will have a timestamp associated with the frame of this
  // scan AND the current time (timestamp of model updates)

  std::vector<cv::Point2f> pos;
  std::vector<cv::Point2f> vel;

  tracker_.add_measurements<Point2fAccess>(pos, vel, 0);


  Eigen::Projective2d T;
  T.setIdentity();

  tracker_.apply_transformation(T);

  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  std::vector<rransac::core::ModelPtr> tracks = tracker_.run();
}

// ----------------------------------------------------------------------------

void RRANSAC::publish_tracks(std::vector<rransac::core::ModelPtr> tracks)
{
  
}

}