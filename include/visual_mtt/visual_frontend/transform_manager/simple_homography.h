#pragma once

#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include "transform_base.h"
#include "common/gpu.h"

namespace visual_frontend {

  /** \class SimpleHomography
  *  \brief Calculates the homography from the previous fram to the current frame.
  */

  class SimpleHomography : public TransformBase
  {
  public:
    SimpleHomography();
    ~SimpleHomography();

    /**
    * \brief Used to initialize the plugin's static parameters.
    * \detail This method is called by TransformManager
    * @param params Contains all the static parameters the plugin needs upon loading.
    * @see common::Params
    * @see TransformManager::LoadPlugins(const std::vector<std::string>& plugin_list, const common::Params& params)
    * @see TransformBase::Initialize(const common::Params& params)
    */
    virtual void Initialize(const common::Params& params) override;

    /**
    * \brief Updates dynamic parameters.
    * \detail This method is called by TransformManager.
    * @param config Struct containing all of the dynamic parameters in cfg/visual_frontend.cfg
    * @see TransformManager::SetParameters(const visual_mtt::visual_frontendConfig& config)
    * @see TransformBase::SetParameters(const visual_mtt::visual_frontendConfig& config)
    */
    virtual void SetParameters(const visual_mtt::visual_frontendConfig& config) override;

    /**
    * \brief Provides a visual display of the homography.
    * \detail It transforms the previous frame into the basis of the current 
    * frame and displays the image differrence between the frames.
    * This method is called by TransformManager when common::System::tuning_ is set to true.
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @see TransformManager::CalculateTransform(common::System& sys)
    * @see TransformBase::DrawTransform(const common::System& sys)
    */
    virtual void DrawTransform(const common::System& sys) override;

    /**
    * \brief Computes the homography from the previous frame to the current frame.
    * \detail This method must set TransformBase::transform_ to the calcualted homography
    * so that the TransformManager can set common::System::transform_. 
    * @param sys Constains all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
    * @return Returns true if the transform is good, otherwise false.
    * @see TransformManager::GetTransform(const common::System& sys)
    * @see TransformBase::TransformManager::GetTransform(const common::System& sys)
    */
    virtual bool GetTransform(const common::System& sys) override;

  private:

    /**
    * \brief Resets member variables when a plugin is toggled from on to off. 
    * \detail This method is called by TransformBase::ShouldReset(bool enable_param).
    * It destroys all created OpenCV windows, and resets method variables.
    * @see TransformBase::Reset()
    */
    virtual void Reset() override;

    /**
    * \brief Destroys all of the OpenCV windows that have been created. 
    */
    void DestroyWindows();


    std::vector<uchar> inlier_mask_;  /**< Homography inlier mask. */
    bool good_transform_;             /**< Indicates if the homography is good. */

    //////////////////////////////////////////////////////////////////////
    // The following is used to only help visualize the transform.

    bool drawn_;                      /**< Indicates if an image is drawn. */

  #if OPENCV_CUDA
    // saved undistorted frames
    cv::cuda::GpuMat frame_u_;        /**< Undistorted current common::System::sd_frame_ */
    cv::cuda::GpuMat frame_u_last_;   /**< Undistorted previous common::System::sd_frame_ */

    // difference image
    cv::cuda::GpuMat frame_difference_; /**< Image difference between previous and current frames */
  #else

    // saved undistorted frames
    cv::Mat frame_u_;                /**< Undistorted current common::System::sd_frame_ */
    cv::Mat frame_u_last_;           /**< Undistorted previous common::System::sd_frame_ */

    // difference image
    cv::Mat frame_difference_;       /**< Image difference between previous and current frames */

  #endif

  };

}