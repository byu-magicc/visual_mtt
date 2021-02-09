#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// rransac source parameters
#include <rransac/common/sources/source_base.h>
#include <rransac/common/measurement/measurement_base.h>

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
    virtual void Initialize(const common::Params& params, const unsigned int source_index) = 0;

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


    rransac::SourceParameters source_parameters_; /**< Source parameters needed by RRANSAC. The child class must fill out the member variables: type_, meas_cov_,  spacial_density_of_false_meas_, probability_of_detection_, gate_probability_, and RANSAC_inlier_probability_.*/
    bool has_velocity_;                           /**< Flag used to indicate if it has velocity. True indicates that it has velocity. */
    bool source_parameters_changed_; /**< Used to indicate if any of the source parameters changed in the SetParameters function. If they did, then rransac 
                                          needs to change the corresponding parameters. */

    std::vector<cv::Point2f> meas_pos_; /**< Possible object's position in the image frame. */
    std::vector<cv::Point2f> meas_vel_; /**< Possible object's velocity in the image frame. */

    common::FrameRefVector frames_required_;            /**< Boolean array defining which frames are required by a measurement plugin during runtime. */
#if OPENCV_CUDA
    common::CUDAFrameRefVector cuda_frames_required_;   /**< Boolean array defining which CUDA frames are required by a measurement plugin during runtime. */
#endif
  protected:

    /**
    * 
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by MeasurementBase::ShouldReset(bool enable_param).
    */
    virtual void Reset() = 0;


    /**
    * < Struct containing the necessary information to save an image.
    * @see common::PictureParams
    */
    common::PictureParams pic_params_;

  };
}

