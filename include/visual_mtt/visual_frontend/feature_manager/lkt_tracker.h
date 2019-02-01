#pragma once

// STD libraries
#include <cmath>

#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include "visual_frontend/feature_manager/feature_base.h"

namespace visual_frontend {

#ifdef OPENCV_CUDA
  typedef cv::cuda::CornersDetector cvFeatureDetector_t;
#else
  typedef cv::GFTTDetector cvFeatureDetector_t;
#endif


  /** \class LKTTracker
  * \brief A Feature Matcher Plugin that uses Lucas-Kanade optical 
    flow method to find matched features.
  */
  class LKTTracker : public FeatureBase
  {
  public:
    LKTTracker();
    ~LKTTracker();

    /**
    * \brief Used to initialize a the plugin's static parameters.
    * \detail This method is called by FeatureManager
    * @param params Contains all the static parameters the plugin needs upon loading.
    * @see common::Params
    * @see FeatureManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
    * @see FeatureBase::Initialize(const common::Params& params)
    */
    virtual void Initialize(const common::Params& params) override;

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by FeatureManager.
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    * @see FeatureManager::SetParameters(const visual_mtt::visual_frontendConfig& config)
    * @see FeatureBase::SetParameters(const visual_mtt::visual_frontendConfig& config)
    */
    virtual void SetParameters(const visual_mtt::visual_frontendConfig& config) override;

    /**
    * \brief Draws and displays the matched features.
    * \detail This method is called by FeatureManager when common::System::tuning_ is set to true.
    * It allows the user to visual see the optical flow between the
    * previous frame and the current frame.
    * @param sys Constains all the data needed for the different managers, plugins, VisualFrontend, and RRANSAC.
    * @see FeatureManager::FindCorrespondences(common::System& sys)
    * @see FeatureBase::DrawFeatures(const common::System& sys)
    */
    virtual void DrawFeatures(const common::System& sys) override;

    /**
    * \brief Find distorted matched features from two sequential images.
    * \detail This method is called by FeatureManager.
    * It uses common::System::sd_frame_ to extract the features
    * that it will match using optical flow.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @return Returns true if the matched features are good.
    * @see FeatureManager::FindCorrespondences(common::System& sys)
    * @see FeatureBase::FindCorrespondences(const common::System& sys)
    */
    virtual bool FindCorrespondences(const common::System& sys) override;
    
  private:

    /**
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by FeatureBase::ShouldReset(bool enable_param).
    * It destroys all created OpenCV windows, and resets method variables.
    * @see FeatureBase::Reset()
    */
    virtual void Reset() override;

    cv::Ptr<cvFeatureDetector_t> InitGftt(int max_features = 0); /**< Create a feature detector object */
    
    /**
    * \detail Wrapper methods to calculate LK optical flow and to detect features.
    * This method is called by LKTTracker::FindCorrespondences(const common::System& sys)
    * @param mono Grayscale image of common::System::sd_frame_
    * @param curr_features Features found in the current mono frame using opitical flow.
    * @param valid Indicates which curr_features are valid.
    * @see LKTTracker::FindCorrespondences(const common::System& sys)
    */
    void CalculateFlow(const cv::Mat& mono, std::vector<cv::Point2f>& curr_features, std::vector<unsigned char>& valid,const common::System& sys);
    
    /**
    * \brief Uses OpenCV's good features to track to extract features from the image.
    * \detial This method is called by LKTTracker::FindCorrespondences(const common::System& sys)
    * @param mono Grayscale image of common::System::sd_frame_
    * @param features Extracted features from the current mono frame
    * @param mask The common::System::undistorted_region_mask_.
    * @see LKTTracker::FindCorrespondences(const common::System& sys)
    */
    void DetectFeatures(const cv::Mat& mono, std::vector<cv::Point2f>& features, const cv::Mat& mask);
    
    /**
    * \brief Sets the maximum number of features the cv::Ptr<cvFeatureDetector_t> should find.
    * \detail This is called by LKTTracker::SetParameters(const visual_mtt::visual_frontendConfig& config)
    * This will either set the cv::Ptr<cvFeatureDetector_t> parameter or will
    * have to recreate the object to set the parameter.
    * This also sets LKTTracker::max_features_.
    * @param max_features The maximum numer of features to be detected.
    */
    void SetMaxFeatures(int max_features);

    /**
    * \brief Destroys all of the OpenCV windows that have been created. 
    */
    void DestroyWindows();

    // Feature Detector
    cv::Ptr<cvFeatureDetector_t> gftt_detector_;
    double corner_quality_;         /**< GFTT: Corner Quality param of GFFT */
    double corner_quality_alpha_;   /**< GFTT: Parameter of alpha-filter to adaptively tune corner quality */
    double max_features_;           /**< GFTT: Max number of features to be extracted. */

    // Image parameters
    bool drawn_;                    /**< Indicates if an image is drawn. */
    bool first_image_;              /**< Indicates if an image has been processed. */

    // Optical Flow
    std::vector<cv::Mat> last_pyramids_;  /**< Optical Flow: Primarily for non-CUDA */
    cv::Mat last_mono_;                   /**< Optical Flow: Primarily for CUDA */
    cv::Size pyramid_size_;    
    cv::TermCriteria kltTerm_;            /**< Optical Flow: Termination criteria */
  };

}