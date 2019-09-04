#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/measurement_manager/measurement_base.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

/** \class ColorDetector
* \brief Find point measurements of certain color and sizes. 
*  This class will convert a RGB image to HSV. It will use the HSV thresholds the user provides
* to represent the color of interest as white (255) and all other colors as black (0). The
* thresholded image will be passed to a cv::SimpleBlobDetector to detects all white shapes 
* of a certain size. The size of the blob is determined by parameters the user sets. The
* detected blobs are represented as point measurements which are given to the measurement
* manager.
*/
class ColorDetector: public MeasurementBase 
{

public:
  ColorDetector();
  ~ColorDetector();

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
  * \brief Core method to generate point measurements from color blobs.
  * \detail It detects color blobs of a certain size and color. These color blobs are
  * represented by point measurements.
  * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
  */
  bool GenerateMeasurements(const common::System& sys) override;



private:

   /**
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by MeasurementBase::ShouldReset(bool enable_param).
    * It destroys all created OpenCV windows, and resets method variables.
    * @see MeasurementBase::ShouldReset(bool enable_param)
    */
    void Reset() override;  

    /**
    * \brief Destroys all of the OpenCV windows that have been created. 
    */
    void DestroyWindows();

    bool first_image_; /**< Indicates if FeatureMotion::GenerateMeasurements(const common::System& sys)
                            is being called for the first time */

    bool drawn_; /**< At least one cv window exists. */

    // HSV Thresholds
    unsigned int min_hue_;           /**< Minumum hue threshold */
    unsigned int max_hue_;           /**< Maximum hue threshold */
    unsigned int min_sat_;           /**< Minumum sat threshold */
    unsigned int max_sat_;           /**< Maximum sat threshold */
    unsigned int min_val_;           /**< Minumum val threshold */
    unsigned int max_val_;           /**< Maximum val threshold */
    unsigned int morph_iterations_;  /**< The number of times the image is eroded and dilated during morphological opening and closing */
    unsigned int morph_kernel_;      /**< The kernel size of the erosion and dilation during morphological opening and closing */

    cv::SimpleBlobDetector::Params params_; /**< A struct that contains the parameters for the simple blob detector */
    cv::Ptr<cv::SimpleBlobDetector> detector_;  /**< The blob detector */

    // Images
    cv::Mat HSV_img_;               /**< HSV Image */
    cv::Mat thresholded_img_;       /**< The color shade of interest will be represented as white */
    cv::Mat inv_img_;               /**< Inverse color of @see ColorDetector::thresholded_img_ */

    std::vector<cv::KeyPoint> keypoints_;  
    std::vector<cv::Point2f> d_meas_pos_;  /**< Point measurements obtained from the distorted image */

    std::string cv_window_name_;    /**< The name of the open cv window that shows threshold filtering */
};
}