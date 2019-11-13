#pragma once

#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// Feature Mature base plugin
#include "feature_base.h"

// Pluginlib
#include <pluginlib/class_loader.h>

namespace visual_frontend {

  /** \class FeatureManager
  * \breif Manages all of the Feature Matcher Plugins.
  * \detail This class is responsible for managing all of the Feature Matcher Plugins.
  * It loads them as indicated by plugin whitelist, sets their 
  * parameters, and calls them to find matching features between frames.
  */

  class FeatureManager
  {
  public:
    FeatureManager();
    ~FeatureManager();

    /**
    * \brief Updates dynamic parameters.
    * \detail ROS rqt_reconfigure allows some parameters to adjust dynamically.
    * Any parameter that adjusts dynamically is passed into this method
    * to update the manager's and plugin's dynamic parameters.
    * This method is called by VisualFrontend.
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    */
    void SetParameters(visual_mtt::visual_frontendConfig& config);

    /**
    * \brief Loads all of the Feature Matcher Plugins indicated by the plugin whitelist.
    * \detail Once a plugin is loaded, it will call the plugin's initialize method.
    * This method is called by VisualFrontend
    * @param plugin_list A List of all the Feature Matcher Plugins to load.
    * @param params Contains all the parameters in the static parameter yaml file. 
    * @see FeatureBase::Initialize(const common::Params& params)
    */
    void LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params);


    /**
    * \brief Calls all of the loaded plugins to find matching distorted features 
    * between the previous and current frame.
    * \detail If the called plugin returns good matches, the matches are added;
    * otherwise they are discarded. 
    * If good features were found then the Common::System::good_features_
    * flag is set and the distorted features are undistorted.
    * If common::System::tuning_ is enabled, the matched features will be drawn.
    * This method is called by VisualFrontend
    * @param sys Constains all the data needed for the different managers, plugins, VisualFrontend, and RRANSAC.
    */
    void FindCorrespondences(common::System& sys);

    common::FrameRefVector frames_required_;            /**< Boolean array defining which frames are required by all feature plugins during runtime. */
#if OPENCV_CUDA
    common::CUDAFrameRefVector cuda_frames_required_;   /**< Boolean array defining which CUDA frames are required by all feature plugins during runtime. */
#endif
   private:

    bool plugins_loaded_; /**< Indicates if the plugins have been loaded */

    
    pluginlib::ClassLoader<FeatureBase> plugin_loader_; /**< Plugin loader */
    std::vector<boost::shared_ptr<FeatureBase>> feature_matchers_; /**< Feature Matcher Loaded Plugins */

  };

}