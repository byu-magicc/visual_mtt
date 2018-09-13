#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/measurement_manager/measurement_base.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

  /** \class FeatureMotion
  * \brief Uses matched features provided by FeatureManager to measure apparent motion.
  */
  class FeatureMotion: public MeasurementBase
  {
  public:
    FeatureMotion();
    ~FeatureMotion();

   /**
    * \brief Used to initialize a plugin's static parameters.
    * \detail This method is called by MeasurementManager
    * @param params Contains all the static parameters the plugin needs upon loading.
    * @see common::Params
    * @see MeasurementManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
    * @see MeasurementBase::Initialize(const common::Params& params)
    */
    void Initialize(const common::Params& params) override;

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by MeasurementManager.
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    * @see MeasurementManager::SetParameters(const visual_mtt::visual_frontendConfig& config)
    * @see MeasurementBase::SetParameters(const visual_mtt::visual_frontendConfig& config)
    */
    void SetParameters(const visual_mtt::visual_frontendConfig& config) override;
    
    /**
    * \brief Draws the measurements on an image and displays it.
    * \detail The generated measurements are drawn to a clone of the current image
    * and displayed.
    * This method is called by MeasurementManager when common::System::tuning_ is set to true.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @see MeasurementManager::CalculateTransform(common::System& sys)
    * @see MeasurementBase::DrawMeasurements(const common::System& sys)
    */
    void DrawMeasurements(const common::System& sys) override;



    /**
    * \brief Core method to generate position and velocity measurements from a frame.
    * \detail It calculates the normalized pixel velocity between matched features, and
    * the velocities greater than FeatureMotion::velocity_floor_ and less than
    * FeatureMotion::velocity_ceiling_ are treated as object measurements.
    * This method is called by MeasurementManager.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @return Returns true if the transform is good, otherwise false.
    * @see TransformManager::GetTransform(const common::System& sys)
    * @see TransformBase::GenerateMeasurements(const common::System& sys)
    */
    bool GenerateMeasurements(const common::System& sys) override;
    

  private:

    /**
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by FeatureBase::ShouldReset(bool enable_param).
    * It destroys all created OpenCV windows, and resets method variables.
    * @see TransformBase::Reset()
    */
    void Reset() override;    

    /**
    * \brief Destroys all of the OpenCV windows that have been created. 
    */
    void DestroyWindows();

    bool first_image_; /**< Indicates if FeatureMotion::GenerateMeasurements(const common::System& sys)
                            is being called for the first time */

    double velocity_floor_; /**< Minimum measurement velocity threshold */
    double velocity_ceiling_; /**< Maximum measurement velocity threshold */

    bool drawn_; /**< At least one cv window exists. */
  };

}