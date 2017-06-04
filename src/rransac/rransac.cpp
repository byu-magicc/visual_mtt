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

void RRANSAC::callback(const visual_mtt2::RRANSACScanPtr& rransac_scan)
{

  // retrieve good models and publish
  // the message will have a timestamp associated with the frame of this
  // scan AND the current time (timestamp of model updates)

  // Access the homography from the ROS message, convert to Projective2d, and give to R-RANSAC
  Eigen::Matrix3f H = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(rransac_scan->homography.data());
  Eigen::Projective2d T(H.cast<double>());
  tracker_.apply_transformation(T);
  
  // Add each source's measurements along with its ID to the R-RANSAC Tracker
  for (auto src = rransac_scan->sources.begin(); src != rransac_scan->sources.end(); src++)
    if (src->dimensionality == 2)
      tracker_.add_measurements<ROSVec2fAccess>(src->positions, src->velocities, src->id);
    else if (src->dimensionality == 3)
      tracker_.add_measurements<ROSVec3fAccess>(src->positions, src->velocities, src->id);

  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  std::vector<rransac::core::ModelPtr> tracks = tracker_.run();
}

// ----------------------------------------------------------------------------

void RRANSAC::publish_tracks(std::vector<rransac::core::ModelPtr> tracks)
{
  
}

}