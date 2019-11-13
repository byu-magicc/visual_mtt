#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// Transform
#include "transform_base.h"

// Pluginlib
#include <pluginlib/class_loader.h>

namespace visual_frontend {

  /** \class TransformManager
  * \brief Manages all of the Transform Method Plugins.
  * \detail This class is responsible for managing all of the Transform Method Plugins.
  * It loads them as indicated by plugin whitelist, sets their 
  * parameters, and calls them to find the transform from the previous to the current frame.
  */

  class TransformManager
  {
  public:
    TransformManager();
    ~TransformManager();

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
    * \brief Loads all of the Transform Method Plugins indicated by the plugin whitelist.
    * \detail Once a plugin is loaded, it will call the plugin's initialize method.
    * This method is called by VisualFrontend
    * @param plugin_list A List of all the Transform Method Plugins to load.
    * @param params Contains all the parameters in the static parameter yaml file. 
    * @see TransformBase::Initialize(const common::Params& params)
    */
    void LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params);



    /**
    * \brief Calls all of the loaded plugins to calculate a transform
    * \detail from the previous to the current frame.
    * If more than one transform method is loaded and enabled, it will
    * only calculate the transform for the first method in the data
    * structure TransformManager::transform_methods_ that is loaded, enabled and
    * computes a common::System::good_transform_. 
    * If common::System::tuning_ is enabled, transforms will be visually represented.
    * This method is called by VisualFrontend
    * @param sys Constains all the data needed for the different managers, plugins, VisualFrontend, and RRANSAC.
    */
    void CalculateTransform(common::System& sys);

    common::FrameRefVector frames_required_;            /**< Boolean array defining which frames are required by all transform plugins during runtime. */
#if OPENCV_CUDA
    common::CUDAFrameRefVector cuda_frames_required_;   /**< Boolean array defining which CUDA frames are required by all transform plugins during runtime. */
#endif
  private:

    bool plugins_loaded_; /**< Indicates if the plugins have been loaded */

    pluginlib::ClassLoader<TransformBase> plugin_loader_; /**< Plugin loader */
    std::vector<boost::shared_ptr<TransformBase>> transform_methods_; /**< Transform Method Loaded Plugins */

  };

}