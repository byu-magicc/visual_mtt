#include "visual_frontend/measurement_manager/measurement_manager.h"

namespace visual_frontend {

MeasurementManager::MeasurementManager() :
    plugin_loader_("visual_mtt", "visual_frontend::MeasurementBase")

{
  plugins_loaded_ = false;
}

// ----------------------------------------------------------------------------

MeasurementManager::~MeasurementManager()
{
  // destroy plugins
  measurement_sources_.clear();
}

// ----------------------------------------------------------------------------

void MeasurementManager::SetParameters(visual_mtt::visual_frontendConfig& config)
{
  // update parameters for each source
  for (int i=0; i<measurement_sources_.size(); i++) 
    measurement_sources_[i]->SetParameters(config);
}

// ----------------------------------------------------------------------------

void MeasurementManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
{

  if (plugins_loaded_) {
    ROS_WARN_STREAM("The Measurement plugins are already loaded");
    return;
  }

  // See if there are any plugins to load
  if (plugin_list.size() > 0) {

    // Try to load each plugin
    for (auto& plugin_name : plugin_list)
    {
      try
      {
        measurement_sources_.emplace_back(plugin_loader_.createInstance(plugin_name));
        ROS_INFO_STREAM("Loaded Measurement plugin: " << plugin_name);
        plugins_loaded_ = true;
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("The Measurement: " <<plugin_name<< " failed to load. Error: "<< ex.what());
      }
    }
  } else {
    ROS_ERROR_STREAM("The Measurement plugin whitelist constains no plugins, so none are loaded.");
  }

  // Initialize plugins
  for (auto&& src : measurement_sources_) 
    src->Initialize(params); 
}

// ----------------------------------------------------------------------------

void MeasurementManager::GenerateMeasurements(common::System& sys)
{

  // Clear History
  sys.ClearMeasurements();

  bool good_measurements = false;

  for (auto&& src : measurement_sources_)
  {
    if (src->enabled_) {
      // generate measurements for this source
      bool good = src->GenerateMeasurements(sys);
      if (good)
      {

        sys.AddMeasurements(src->id_, src->has_velocity_, src->meas_pos_, src->meas_vel_);
      }

      good_measurements = (good || good_measurements);

      // draw measurements for this source
      if (sys.tuning_) src->DrawMeasurements(sys);

    }
  }

  sys.SetMeasurementFlag(good_measurements);

  if (sys.tuning_)
    char keyboard = cv::waitKey(1);

}

}