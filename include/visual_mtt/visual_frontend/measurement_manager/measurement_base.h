#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// Dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// common
#include "common/params.h"
#include "common/system.h"

// Pluginlib
#include <pluginlib/class_list_macros.h>

namespace visual_frontend {

  /** \class MeasurementBase
  * \brief This is the abstract base class for all Measurement Source Plugins.
  * \detail Any Measurement Source Plugin must inherit this class and implement
  * the member methods. The purpose of Measurement Source Plugins
  * is to generate position and/or velocity measurements of possible objects
  * in the image frame and feed them into RRANSAC so that the objects
  * can be tracked. 
  */

  class MeasurementBase
  {
  public:

   /**
    * \brief Used to initialize a plugin's static parameters.
    * \detail This method is called by MeasurementManager
    * to pass in any parameters a Measurement Source Plugin needs
    * once it is loaded. This is necessary because a plugin's 
    * constructor can't be passed any arguments.
    * @param params Contains all the static parameters the plugin needs upon loading.
    * @see common::Params
    * @see MeasurementManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
    */
    virtual void Initialize(const common::Params& params) = 0;

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by MeasurementManager.
    * ROS rqt_reconfigure allows some parameters to adjust dynamically.
    * Any parameter that adjusts dynamically is passed into the method
    * to update the plugin's dynamic parameters.
    * This method must call MeasurementBase::ShouldReset(bool enable_param)
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    * @see MeasurementManager::SetParameters(const visual_mtt::visual_frontendConfig& config)
    */
    virtual void SetParameters(const visual_mtt::visual_frontendConfig& config) = 0;
    
    /**
    * \brief Draws the measurements on an image and displays it.
    * \detail This method is called by MeasurementManager when common::System::tuning_ is set to true.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @see MeasurementManager::CalculateTransform(common::System& sys)
    */
    virtual void DrawMeasurements(const common::System& sys) = 0;

    /**
    * \brief Indicates when MeasurementBase::Reset() should be called.
    * \detail When a plugin is toggled from 'on' to 'off'
    * MeasurementBase::Reset() is called. 
    * ShouldReset needs to be called by the inherited
      MeasurementBase::SetParameters(const visual_mtt::visual_frontendConfig& config)
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
    * \brief Core method to generate position and/or velocity measurements from a frame.
    * \detail This method is called by MeasurementManager.
    * This method must set MeasurementBase::meas_pos_ and MeasurementBase::meas_vel_
    * so that the MeasurementManager can add the measurements to a common::System object.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @return Returns true if the transform is good, otherwise false.
    * @see TransformManager::GetTransform(const common::System& sys)
    */
    virtual bool GenerateMeasurements(const common::System& sys) = 0;
    
    bool enabled_;      /**< Flag used to indicate if the plugin is enabled. 
                             This needs to be set in the inherited 
                             MeasurementBase::SetParameters()*/
    std::string name_;  /**< A unique identifier for the plugin. Usually the plugin's class name. 
                             Should be initialized in the plugin's constructor.*/   


    int id_;            /**< Each Measurement Source Plugin must have a unique ID. This
                             is used for RRANSAC. Must be defined in the plugin's constructor.*/
    bool has_velocity_; /**< Indicates if the Measurement Source Plugin measures a 
                             possible object's velicty. Must be defined in the plugin's constructor.*/
    double sigmaR_pos_; /**< Standard deviation of measurement's position noise. This 
                             is used for  RRANSAC and must be defined in the plugin's constructor.*/
    double sigmaR_vel_; /**< Standard deviation of measurement's velocity noise. This 
                             is used for  RRANSAC and must be defined in the plugin's constructor.*/

    std::vector<cv::Point2f> meas_pos_; /**< Possible object's position in the image frame. */
    std::vector<cv::Point2f> meas_vel_; /**< Possible object's velocity in the image frame. */

    common::FrameRefVector frames_required_;
    common::CUDAFrameRefVector cuda_frames_required_;

  protected:

    /**
    * 
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by MeasurementBase::ShouldReset(bool enable_param).
    */
    virtual void Reset() = 0;


  };
}

