#pragma once

#include <vector>

// Dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// Pluginlib
#include <pluginlib/class_list_macros.h>

// common
#include <common/params.h>
#include <common/system.h>

namespace visual_frontend {

  /** \class TransformBase
  * This is the abstract base class for all Transform Method Plugins.
  * \detail Any Transform Method Plugin must inherit this class and implement
  * the member methods. The purpose of Transform Method Plugins
  * is to use undistorted matched features provided by 
  * FeatureManager to find the transform
  * from the previous to the current frame. 
  * All Transform Method Plugins are handled by 
  * TransformManager. 
  */

  class TransformBase
  {
  public:

   /**
    * \brief Used to initialize a plugin's static parameters.
    * \detail This method is called by TransformManager
    * to pass in any parameters a Transform Method Plugin needs
    * once it is loaded. This is necessary because a plugin's 
    * constructor can't be passed any arguments.
    * @param params Contains all the static parameters the plugin needs upon loading.
    * @see common::Params
    * @see TransformManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
    */
    virtual void Initialize(const common::Params& params) = 0;

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by TransformManager.
    * ROS rqt_reconfigure allows some parameters to adjust dynamically.
    * Any parameter that adjusts dynamically is passed into the method
    * to update the plugin's dynamic parameters.
    * This method must call TransformBase::ShouldReset(bool enable_param)
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    * @see TransformManager::SetParameters(const visual_mtt::visual_frontendConfig& config)
    */
    virtual void SetParameters(const visual_mtt::visual_frontendConfig& config) = 0;
    
    /**
    * \brief Provides a visual display of the transform.
    * \detail This method is called by TransformManager when common::System::tuning_ is set to true.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @see TransformManager::CalculateTransform(common::System& sys)
    */
    virtual void DrawTransform(const common::System& sys) = 0;

    /**
    * \brief Indicates when TransformBase::Reset() should be called.
    * \detail When a plugin is toggled from 'on' to 'off'
    * TransformBase::Reset() is called. 
    * ShouldReset needs to be called by the inherited
      TransformBase::SetParameters(const visual_mtt::visual_frontendConfig& config)
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
    * \brief Computes the transform from the previous frame to the current frame.
    * \detail This method must set TransformBase::transform_ to the calcualted transform
    * so that the TransformManager can set common::System::transform_. 
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @return Returns true if the transform is good, otherwise false.
    * @see TransformManager::GetTransform(const common::System& sys)
    */
    virtual bool GetTransform(const common::System& sys)=0;

    bool enabled_;      /**< Flag used to indicate if the plugin is enabled. 
                             This needs to be set in the inherited 
                             TransformBase::SetParameters()*/
    std::string name_;  /**< A unique identifier for the plugin. Usually the plugin's class name. 
                             Should be initialized in the plugin's constructor.*/   

    cv::Mat transform_; /**< Transform calculated from the Transform Method Plugin. */

  protected:

    /**
    * 
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by TransformBase::ShouldReset(bool enable_param).
    */
    virtual void Reset() = 0;

  };

}