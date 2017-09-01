#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "common/gpu.h"
#include "visual_frontend/source_manager/measurement_source.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

  class DifferenceImage: public MeasurementSource
  {
  public:
    DifferenceImage();
    ~DifferenceImage();
    void generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform);
    void set_parameters(visual_mtt::visual_frontendConfig& config);
    void set_camera(const cv::Mat& K, const cv::Mat& D);
    void draw_measurements();

    bool first_image_ = true; // see TODO notes in difference_image.cpp

  private:
#if OPENCV_CUDA
    void mask_edges(cv::cuda::GpuMat& difference_raw, cv::cuda::GpuMat& difference_masked, cv::Mat& homography);
#else
    void mask_edges(cv::Mat& difference_raw, cv::Mat& difference_masked, cv::Mat& homography);
#endif

    // untouched input frame
    cv::Mat sd_frame_;

#if OPENCV_CUDA
    // saved undistorted frames
    cv::cuda::GpuMat frame_u_;
    cv::cuda::GpuMat frame_u_last_;
    // intermediate steps
    cv::cuda::GpuMat frame_difference_;
    cv::cuda::GpuMat frame_blur_;
    cv::cuda::GpuMat frame_normalized_;
    cv::cuda::GpuMat frame_threshold_;
    cv::cuda::GpuMat frame_open_;
#else
    // saved undistorted frames
    cv::Mat frame_u_;
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

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;

    // for extra introspective plots
    bool extra_plots_;

    // at least one cv window exists
    bool drawn_;
  };

}