#include "visual_frontend/feature_manager/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)

namespace visual_frontend {

FeatureManager::FeatureManager() :
  plugin_loader_("visual_mtt", "visual_frontend::FeatureBase")
{
  plugins_loaded_ = false;
  // Set initial required feature frames and CUDA frames to false
  for(int i = 0; i < common::num_frame_types_; i++)
    {
      frames_required_[i] = false;
    }
    for(int i = 0; i < common::num_cuda_frame_types_; i++)
    {
      cuda_frames_required_[i] = false;
    }
}

// ----------------------------------------------------------------------------

FeatureManager::~FeatureManager()
{
  // Destroy the plugins.
  feature_matchers_.clear();
}

// ----------------------------------------------------------------------------

void FeatureManager::SetParameters(visual_mtt::visual_frontendConfig& config)
{

  // Set the parameters for each plugin
  for (auto&& src : feature_matchers_)
      src->SetParameters(config);

}

// ----------------------------------------------------------------------------

void FeatureManager::LoadPlugins(const std::vector<std::string>& plugin_list,const common::Params& params)
{
  if (plugins_loaded_) {
  ROS_WARN_STREAM("The FeatureTracker plugins are already loaded");
  return;
  }

  // See if there are any plugins to load
  if (plugin_list.size() > 0) {

    // Try to load each plugin
    for (auto& plugin_name : plugin_list)
    {
      try
      {
        feature_matchers_.emplace_back(plugin_loader_.createInstance(plugin_name));
        ROS_INFO_STREAM("Loaded FeatureTracker plugin: " << plugin_name);
        plugins_loaded_ = true;
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("The FeatureTracker: " <<plugin_name<< " failed to load. Error: "<< ex.what());
      }
    }
  } else {
    ROS_ERROR_STREAM("The FeatureTracker plugin whitelist constains no plugins, so none are loaded.");
  }

  // Initialize plugins
  for (auto&& src : feature_matchers_) 
  {
    src->Initialize(params);  
    // Incorporate required frames from each feature plugin
    for(int i = 0; i < common::num_frame_types_; i++)
    {
      frames_required_[i] = frames_required_[i] | src->frames_required_[i];
    }
    for(int i = 0; i < common::num_cuda_frame_types_; i++)
    {
      cuda_frames_required_[i] = cuda_frames_required_[i] | src->cuda_frames_required_[i];
    }
  }

}

// ----------------------------------------------------------------------------

void FeatureManager::FindCorrespondences(common::System& sys)
{
  // clear history
  sys.ClearMatchedFeatures();

  bool good_features = false;

  for (auto&& src : feature_matchers_)
  {
    if (src->enabled_) {
      // generate measurements for this source
      bool good = src->FindCorrespondences(sys);
      if (good)
        sys.AddMatchedFeatures(src->d_prev_matched_, src->d_curr_matched_);
      good_features = (good || good_features);
      
      // draw measurements for this source
      if (sys.tuning_) src->DrawFeatures(sys);
    }
  }

  sys.SetFeatureFlag(good_features);

  sys.UndistortMatchedFeatures();

  if (sys.tuning_)
    char keyboard = cv::waitKey(1);

}

}