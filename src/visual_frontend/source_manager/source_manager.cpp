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
  if (config.feature_outliers_enabled != feature_motion_)
    changed = true;
  if (config.difference_image_enabled != difference_image_)
    changed = true;

  if (changed)
  {
    // update the enabled/disabled status for each source
    feature_motion_   = config.feature_outliers_enabled;
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

void SourceManager::generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform)
{
  // initialize the scan message
  visual_mtt::RRANSACScan scan;

  for (int i=0; i<measurement_sources_.size(); i++)
  {
    // generate measurements for this source
    measurement_sources_[i]->generate_measurements(
      hd_frame,
      sd_frame,
      homography,
      prev_features,
      next_features,
      good_transform);

    // draw measurements for this source
    if (tuning_)
      measurement_sources_[i]->draw_measurements();

    if (i==0)   // <<<<< HACK HACK HACK to not add difference image to scan
    {
    // create Source message
    visual_mtt::Source src;
    src.id = i;
    src.dimensionality = 2; // TODO: Maybe ask the source what kind of measurements it produces?

    for (int j=0; j<measurement_sources_[i]->features_.size(); j++)
    {
      auto pos = measurement_sources_[i]->features_[j];
      auto vel = measurement_sources_[i]->features_vel_[j];

      visual_mtt::Measurement mpos, mvel;
      mpos.data = {pos.x, pos.y};
      mvel.data = {vel.x, vel.y};

      src.positions.push_back(mpos);
      src.velocities.push_back(mvel);
    }

    // Add source to scan message
    scan.sources.push_back(src);
    }   // <<<<< HACK HACK HACK

  }
  scan_ = scan; // TODO use only scan_ and use "new" up above

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
    measurement_sources_.push_back(std::unique_ptr<FeatureOutliers>(new FeatureOutliers()));
    n_sources_++;
  }
  if (difference_image_)
  {
    measurement_sources_.push_back(std::unique_ptr<DifferenceImage>(new DifferenceImage()));
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