#pragma once

// Dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Common
#include "common/params.h"
#include "common/system.h"
#include "common/gpu.h"

namespace visual_frontend {

  /** \class FeatureBase
  * \brief This is the abstract base class for all Feature Matcher Plugins.
  * \detail Any Feature Matcher Plugin must inherit this class and implement
  * the member methods. The purpose of Feature Matcher Plugins
  * is to match features from the previous
  * to the current frame. These matched features can be used to
  * create a transfrom between frames and generate
  * measurements that are fed into RRANSAC.
  * All Feature Matcher Plugins are handled by 
  * FeatureManager. 
  */

  class FeatureBase
  {
  public:

    /**
    * \brief Used to initialize a plugin's static parameters.
    * \detail This method is called by FeatureManager
    * to pass in any parameters a Feature Matcher Plugin needs
    * once it is loaded. This is necessary because a plugin's 
    * constructor can't be passed any arguments.
    * @param params Contains all the static parameters the plugin needs upon loading.
    * @see common::Params
    * @see FeatureManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
    */
    virtual void Initialize(const common::Params& params) = 0;

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by FeatureManager.
    * ROS rqt_reconfigure allows some parameters to adjust dynamically.
    * Any parameter that adjusts dynamically is passed into the method
    * to update the plugin's dynamic parameters.
    * This method must call FeatureBase::ShouldReset(bool enable_param)
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    * @see FeatureManager::SetParameters(const visual_mtt::visual_frontendConfig& config)
    */
    virtual void SetParameters(const visual_mtt::visual_frontendConfig& config) = 0;
    

    /**
    * \breif Draws and displays the matched features.
    * \detail This method is called by FeatureManager when common::System::tuning_ is set to true.
    * It allows the user to visual see what features are being matched.
    * @param sys Constains all the data needed for the different managers, plugins, VisualFrontend, and RRANSAC.
    * @see FeatureManager::FindCorrespondences(common::System& sys)
    */
    virtual void DrawFeatures(const common::System& sys) = 0;
    
    /**
    * \brief Indicates when FeatureBase::Reset() should be called.
    * \detail When a plugin is toggled from 'on' to 'off'
    * FeatureBase::Reset() is called. 
    * ShouldReset needs to be called by the inherited
      FeatureBase::SetParameters(const visual_mtt::visual_frontendConfig& config)
    * @param enable_param Parameter from visual_mtt::visual_frontendConfig 
            that indicates if the child Feature Matcher should be on or off.
    */
    void ShouldReset(bool enable_param)
    {
      if (enabled_ == true && enable_param == false)
        Reset();
      enabled_ = enable_param;
    }

    /**
    * \brief Find distorted matched features from two sequential images.
    * \detail This method is called by FeatureManager.
    * When it is first called, FeatureBase::d_prev_features_ is empty
    * so it only finds distorted features. On all subsequent calls,
    * it matches found features in the previous frame with features in
    * the current frame. Then it finds new features in the current fram
    * to avoid optical flow drift.
    * This method must set FeatureBase::d_prev_matched_ and 
    * FeatureBase::d_curr_matched_ so that the FeatureManager can set
    * common::System::d_prev_matched_, 
    * common::System::d_curr_matched_, 
    * common::System::ud_prev_matched_, and
    * common::System::ud_curr_matched_. 
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @return Returns true if the matched features are good.
    * @see FeatureManager::FindCorrespondences(common::System& sys)
    */
    virtual bool FindCorrespondences(const common::System& sys) = 0;

    bool enabled_;      /**< Flag used to indicate if the plugin is enabled. 
                             This needs to be set in the inherited 
                             FeatureBase::SetParameters()*/
    std::string name_;  /**< A unique identifier for the plugin. Usually the plugin's class name. 
                             Should be initialized in the plugin's constructor.*/   

    std::vector<cv::Point2f> d_prev_matched_; /**< The distorted matched features from the previous image. */
    std::vector<cv::Point2f> d_curr_matched_; /**< The distorted matched features from the current image. */

    common::FrameRefVector frames_required_;           /**< Boolean array defining which frames are required by a feature plugin during runtime. */
#if OPENCV_CUDA
    common::CUDAFrameRefVector cuda_frames_required_;  /**< Boolean array defining which CUDA frames are required by a feature plugin during runtime. */
#endif
  protected:


    /**
    * \detail Resets member variables when a plugin is toggled from on to off. 
    * This method is called by FeatureBase::ShouldReset(bool enable_param).
    */
    virtual void Reset() = 0;

    std::vector<cv::Point2f> d_prev_features_; /**< The extracted distorted features from the previous */

  };

}