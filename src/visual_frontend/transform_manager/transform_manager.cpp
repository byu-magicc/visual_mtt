#include "visual_frontend/transform_manager/transform_manager.h"

namespace visual_frontend {

TransformManager::TransformManager() : 
    plugin_loader_("visual_mtt", "visual_frontend::TransformBase")
{
  plugins_loaded_ = false;
  // Set initial required transform frames and CUDA frames to false
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

TransformManager::~TransformManager()
{
  // destroy plugins
  transform_methods_.clear();
}

// ----------------------------------------------------------------------------

void TransformManager::SetParameters(visual_mtt::visual_frontendConfig& config)
{
  // update parameters for each transform method
  for (auto&& src : transform_methods_) 
    src->SetParameters(config);
}

// ----------------------------------------------------------------------------

void TransformManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
{

  if (plugins_loaded_) {
    ROS_WARN_STREAM("The Transform plugins are already loaded");
    return;
  }

  // See if there are any plugins to load
  if (plugin_list.size() > 0) {

    // Try to load each plugin
    for (auto& plugin_name : plugin_list)
    {
      try
      {
        transform_methods_.emplace_back(plugin_loader_.createInstance(plugin_name));
        ROS_INFO_STREAM("Loaded Transform plugin: " << plugin_name);
        plugins_loaded_ = true;
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("The Transform: " <<plugin_name<< " failed to load. Error: "<< ex.what());
      }
    }
  } else {
    ROS_ERROR_STREAM("The Transform plugin whitelist constains no plugins, so none are loaded.");
  }

  // Initialize plugins
  for (auto&& src : transform_methods_) 
  {
    src->Initialize(params); 
    // Incorporate required frames from each transform plugin
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

void TransformManager::CalculateTransform(common::System& sys)
{

  // Set Transform to identity
  sys.ClearTransform();
  bool good_transform = false;

  // Iterate through all of the transform method plugins.
  for (auto&& src : transform_methods_) {

    if (src->enabled_) {

      // Find the transfrom that relates the previous frame to the current frame
      // and expose that transfrom as a public data member.
      good_transform = src->GetTransform(sys);

      if(sys.tuning_)
          src->DrawTransform(sys);      
      if (good_transform) 
      {
        sys.SetTransform(src->transform_);
        break;      
      }
    }
  }

  sys.SetTransformFlag(good_transform);

}


}