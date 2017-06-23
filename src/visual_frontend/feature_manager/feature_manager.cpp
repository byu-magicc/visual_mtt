#include "visual_frontend/feature_manager/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)

namespace visual_frontend {

FeatureManager::FeatureManager()
{
  
  // Initialize feature tracker with default type
  set_tracker(KLT_TRACKER);
}

// ----------------------------------------------------------------------------

void FeatureManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{

  // Check to see if feature tracker type has changed
  if (feature_tracker_type_ != static_cast<enum FeatureTrackerType>(config.feature_type))
    set_tracker(static_cast<enum FeatureTrackerType>(config.feature_type));

  feature_tracker_->set_max_features(config.points_max);
}

// ----------------------------------------------------------------------------

void FeatureManager::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();
}

// ----------------------------------------------------------------------------

void FeatureManager::find_correspondences(cv::Mat& img)
{

  // clear history
  prev_matched_.clear();
  next_matched_.clear();

  // Find the feature correspondences across current and previous frame
  // and expose as public data members: `prev_matched_` and `next_matched_`.
  feature_tracker_->find_correspondences(img, prev_matched_, next_matched_);

  // compensate for lens distortion and project onto normalized image plane
  if (prev_matched_.size() > 0 && next_matched_.size() > 0)
  {
    cv::undistortPoints(prev_matched_, prev_matched_, camera_matrix_, dist_coeff_);
    cv::undistortPoints(next_matched_, next_matched_, camera_matrix_, dist_coeff_);
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void FeatureManager::set_tracker(enum FeatureTrackerType type)
{

  // Create a instance of a private node handle
  ros::NodeHandle nh("~");

  if (type == KLT_TRACKER)
  {

    // Retrieve the needed parameters for the LKT Tracker
    double cqual, cqual_min, cqual_max, cqual_alpha;
    int pyramid_size;
    nh.param<double>("corner_quality",       cqual,        0.03 );
    nh.param<double>("corner_quality_min",   cqual_min,    0.03 );
    nh.param<double>("corner_quality_max",   cqual_max,    0.05 );
    nh.param<double>("corner_quality_alpha", cqual_alpha,  0.999);
    nh.param<int>   ("pyramid_size",         pyramid_size, 21   );

    // Create a new feature tracker
    feature_tracker_ = std::make_shared<LKTTracker>(cqual, cqual_min, cqual_max, cqual_alpha, pyramid_size);
  }
  else if (type == ORB_BF_MATCHER)
  {
    ROS_WARN("ORB_BF_MATCHER not implemented");
  }

  // Store what type of feature tracker we are for later
  feature_tracker_type_ = type;
}

}