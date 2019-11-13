#pragma once

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>

#include <rransac/tracker.h>

#include "visual_frontend/accessors.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// available measurement generation methods
#include "visual_frontend/measurement_manager/measurement_base.h"

// Pluginlib
#include <pluginlib/class_loader.h>

namespace visual_frontend {


  /** \class MeasurementManager
  * \brief Manages all of the Measurement Source Plugins.
  * \detail This class is responsible for managing all of the Measurement Source Plugins.
  * It loads them as indicated by plugin whitelist, sets their 
  * parameters, and calls them to generate measurements of possible object's 
  * position and velocity in the normalized image frame.
  */
  class MeasurementManager
  {
  public: 
    MeasurementManager();
    ~MeasurementManager();

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by VisualFrontend.
    * ROS rqt_reconfigure allows some parameters to adjust dynamically.
    * Any parameter that adjusts dynamically is passed into this method
    * to update the manager's and plugin's dynamic parameters.
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    */
    void SetParameters(visual_mtt::visual_frontendConfig& config);

    /**
    * \brief Loads all of the Measurement Source Plugins indicated by the plugin whitelist.
    * \detail Once a plugin is loaded, it will call the plugin's initialize method.
    * This method is called by VisualFrontend
    * @param plugin_list A List of all the Transform Method Plugins to load.
    * @param params Contains all the parameters in the static parameter yaml file. 
    * @see MeasurementBase::Initialize(const common::Params& params)
    */
    void LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params);

    /**
    * \brief Calls all of the loaded plugins to generate measurements.
    * \detail If common::System::tuning_ is enabled, the measurements will be drawn.
    * This method is called by VisualFrontend
    * @param sys Constains all the data needed for the different managers, plugins, VisualFrontend, and RRANSAC.
    */
    void GenerateMeasurements(common::System& sys);

    // TODO make this private. See visual_frontend::callback_reconfigure
    std::vector<boost::shared_ptr<MeasurementBase>> measurement_sources_; /**< Measurement Source Loaded Plugins */

    common::FrameRefVector frames_required_;            /**< Boolean array defining which frames are required by all measurement plugins during runtime. */
#if OPENCV_CUDA
    common::CUDAFrameRefVector cuda_frames_required_;   /**< Boolean array defining which CUDA frames are required by all measurement plugins during runtime. */
#endif
  private:

    bool plugins_loaded_; /**< Indicates if the plugins have been loaded */

    pluginlib::ClassLoader<MeasurementBase> plugin_loader_; /**< Plugin loader */

  };

}