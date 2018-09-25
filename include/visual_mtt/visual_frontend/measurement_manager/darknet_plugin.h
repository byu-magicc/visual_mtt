#pragma once

// std libraries
#include <iostream>
#include <thread>
#include <mutex>

// Opencv
#include <opencv2/opencv.hpp>

// VMTT
#include "visual_frontend/measurement_manager/measurement_base.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// darknet wrapper 
#include "darknet_wrapper/common/bounding_boxes.h"
#include "darknet_wrapper/YoloObjectDetector.h"
#include "darknet_wrapper/common/dynamic_params.h"

namespace visual_frontend {

  /** \class DarknetPlugin
  * \brief Finds and classifies objects using the YOLO network.
  */
  class DarknetPlugin: public MeasurementBase
  {
  public:
    DarknetPlugin();
    ~DarknetPlugin();

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

    // The darknet_wrapper calls this function when an image is done beign processed.
    // The results are then passed into this function.
    void YoloCallback(const cv::Mat& img, const darknet_wrapper::common::BoundingBoxes& boxes, const int& seq);

    /**
    * \brief Destroys all of the OpenCV windows that have been created. 
    */
    void DestroyWindows();


    darknet_wrapper::YoloObjectDetector yolo; /** < Darknet Yolo object */

    cv::Mat drawn_img_;  /** < Image with the detections drawn on it */
    int img_seq_;        /** < The sequence number for the image */
    int sequence_;       /** < Counter for the image sequence number */
    bool drawn_;         /**< At least one cv window exists. */
    darknet_wrapper::common::BoundingBoxes boxes_;
    std::vector<cv::Point2f> d_curr_points_;
    std::mutex yolo_callback_mutex_; /**< d_curr_points_ is touched by two threads. This mutex helps handle r/w access. */



  };

}