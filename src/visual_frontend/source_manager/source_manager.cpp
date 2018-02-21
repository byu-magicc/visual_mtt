#include "visual_frontend/source_manager/source_manager.h"

namespace visual_frontend {

SourceManager::SourceManager()
{
  // populate the vector of sources according to the default settings
  set_sources();
}

// ----------------------------------------------------------------------------

void SourceManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // check to see if any of the source selections have changed
  bool changed = false;
  if (config.feature_motion_enabled != feature_motion_)
    changed = true;
  if (config.difference_image_enabled != difference_image_)
    changed = true;

  if (changed)
  {
    // update the enabled/disabled status for each source
    feature_motion_   = config.feature_motion_enabled;
    difference_image_ = config.difference_image_enabled;

    // repopulate the vector of sources
    set_sources();
  }

  // update parameters for each source
  for (int i=0; i<measurement_sources_.size(); i++)
    measurement_sources_[i]->set_parameters(config);
}

// ----------------------------------------------------------------------------

void SourceManager::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  // update local camera parameters
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();

  // set the camera parameters for each source
  for (int i=0; i<measurement_sources_.size(); i++)
    measurement_sources_[i]->set_camera(K, D);
}

// ----------------------------------------------------------------------------

void SourceManager::feed_rransac(rransac::Tracker& tracker,
                      cv::Mat& hd_frame, cv::Mat& sd_frame,
                      cv::Mat& homography, bool good_transform,
                      std::vector<cv::Point2f>& prev_features,
                      std::vector<cv::Point2f>& next_features)
{

  for (auto&& src : measurement_sources_)
  {
    // generate measurements for this source
    src->generate_measurements(
      hd_frame, sd_frame,
      homography, good_transform,
      prev_features, next_features);

    // draw measurements for this source
    if (tuning_) src->draw_measurements();

    // Add each source's measurements along with its ID to the R-RANSAC Tracker
    if (src->has_velocity_)
      tracker.add_measurements<CVPoint2fAccess>(src->features_, src->features_vel_, src->id_);
    else if (!src->has_velocity_)
      tracker.add_measurements<CVPoint2fAccess>(src->features_, src->id_);

  }

  if (tuning_)
    char keyboard = cv::waitKey(1);

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void SourceManager::set_sources()
{
  // clear the vector
  measurement_sources_.clear();
  n_sources_ = 0;

  // populate the sources vector according to the current configuration
  if (feature_motion_)
  {
    measurement_sources_.emplace_back(std::make_shared<FeatureMotion>());
    n_sources_++;
  }
  if (difference_image_)
  {
    measurement_sources_.emplace_back(std::make_shared<DifferenceImage>());
    n_sources_++;
  }

  // check for "no sources" error
  if (n_sources_==0)
    ROS_ERROR("source manager: no measurement sources are enabled");

  // set the camera parameters for each source
  for (int i=0; i<measurement_sources_.size(); i++)
    measurement_sources_[i]->set_camera(camera_matrix_, dist_coeff_);
}

}