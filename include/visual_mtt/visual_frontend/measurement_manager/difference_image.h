#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "common/gpu.h"
#include "visual_frontend/measurement_manager/measurement_base.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

  /** \class DifferenceImage
  * Uses Image Differencing to measure apparent motion.
  */
  class DifferenceImage: public MeasurementBase
  {
  public:
    DifferenceImage();
    ~DifferenceImage();


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
    * \brief Core method to generate position measurements from a frame.
    * \detail It transform the previous common::System::sd_frame_ into the basis
    * of the current common::System::sd_frame_ and used image differenceing
    * with other techniques to find possible moving objects.
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

    /**
    * \brief Destroys all of the extra OpenCV windows that have been created. 
    */
    void DestroyExtraWindows();

#if OPENCV_CUDA
    void MaskEdges(cv::cuda::GpuMat& difference_raw, cv::cuda::GpuMat& difference_masked, cv::Mat& transform);
#else
    void MaskEdges(cv::Mat& difference_raw, cv::Mat& difference_masked, cv::Mat& transform);
#endif

    bool extra_plots_; /**< Indicates if the extra plots should be drawn */

    cv::Mat camera_matrix_; /**< Camera matrix converted to CV_64FC3 */

    // at least one cv window exists
    bool drawn_;                 /**< Indicates if main plot window exist. */
    bool extra_plots_drawn_;     /**< Indicates if extra plots window exist. */
    bool first_image_ = true;    /**< Indicates if DifferenceImage::GenerateMeasurements(const common::System& sys)
                                     is being called for the first time */


#if OPENCV_CUDA
    // saved undistorted frames
    cv::cuda::GpuMat frame_u_last_;      /**< Previous undistorted common::System::sd_frame_ */
    // intermediate steps
    cv::cuda::GpuMat frame_difference_;  /**< Difference image between DifferenceImage::frame_u_ and  DifferenceImage::frame_u_last_*/
    cv::cuda::GpuMat frame_blur_;        /**< Blurred DifferenceImage::frame_difference_. */
    cv::cuda::GpuMat frame_normalized_;  /**< Normalized DifferenceImage::frame_normalized_. */
    cv::cuda::GpuMat frame_threshold_;   /**< Threshold DifferenceImage::frame_blur_ by DifferenceImage::threshold_ */
    cv::cuda::GpuMat frame_open_;        /**< Erroded and Dilated DifferenceImage::frame_threshold_. */
#else
    // saved undistorted frames
    cv::Mat frame_u_last_;
    // intermediate steps
    cv::Mat frame_difference_;
    cv::Mat frame_blur_;
    cv::Mat frame_normalized_;
    cv::Mat frame_threshold_;
    cv::Mat frame_open_;
#endif
    cv::Mat frame_contours_;
    cv::Mat frame_points_;

    // image corners for difference mask
    bool size_known_ = false;
    cv::Size size_;
    double border_ = 5;
    std::vector<cv::Point2f> corners_;

    // contours saved as members for plotting
    std::vector<std::vector<cv::Point>> contours1_, contours2_;

    // gaussian blur parameters
#if OPENCV_CUDA
    cv::Ptr<cv::cuda::Filter> filter_gauss_;
#endif
    cv::Size blur_kernel_;
    double blur_sigma_;

    // morphology open parameters
#if OPENCV_CUDA
    cv::Ptr<cv::cuda::Filter> filter_open_;
#endif
    cv::Mat element_;
    int morph_iterations_;
    double threshold_;
    int min_complexity_;
    int max_complexity_;

  };
}