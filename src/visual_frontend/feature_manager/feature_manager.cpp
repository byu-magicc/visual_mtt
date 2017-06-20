#include "visual_frontend/feature_manager/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)

namespace visual_mtt {

FeatureManager::FeatureManager()
{

  // Initialize feature tracker
  set_tracker();

}

// ----------------------------------------------------------------------------

void FeatureManager::set_parameters(visual_mtt2::visual_frontendConfig& config)
{
  feature_tracker_->set_max_features(config.points_max, config.points_target);
}

// ----------------------------------------------------------------------------

void FeatureManager::find_correspondences(cv::Mat& img)
{

  // clear history
  prev_matched_.clear();
  next_matched_.clear();

  feature_tracker_->find_correspondences(img, prev_matched_, next_matched_);

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void FeatureManager::set_tracker()
{

  ros::NodeHandle nh("~");

  // if (detector_type == asdf)
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

}

}