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
  if (config.dnn_activations_enabled != dnn_activations_)
    changed = true;

  if (changed)
  {
    // update the enabled/disabled status for each source
    feature_motion_   = config.feature_outliers_enabled;
    difference_image_ = config.difference_image_enabled;
    dnn_activations_  = config.dnn_activations_enabled;

    // repopulate the vector of sources
    set_sources();
  }

  // call the param update for each source
  for (int i=0; i<measurement_sources_.size(); i++)
    measurement_sources_[i]->set_parameters(config);
}

// ----------------------------------------------------------------------------

void SourceManager::generate_measurements()
{
  std::cout << "pretending to generate measurements for all sources" << std::endl;
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
    ROS_WARN("source manager: difference image not implemented");
    // n_sources_++;
  }
  if (dnn_activations_)
  {
    ROS_WARN("source manager: dnn activations not implemented");
    // n_sources_++;
  }

  // check for no sources error
  if (n_sources_==0)
    ROS_ERROR("source manager: no measurement sources are enabled");
}

}