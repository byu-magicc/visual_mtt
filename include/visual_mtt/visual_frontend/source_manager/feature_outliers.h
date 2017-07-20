#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source_manager/measurement_source.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

  class FeatureOutliers: public MeasurementSource
  {
  public:
    FeatureOutliers();
    ~FeatureOutliers();
    void generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform);
    void set_parameters(visual_mtt::visual_frontendConfig& config);
    void set_camera(const cv::Mat& K, const cv::Mat& D);
    void draw_measurements();

    bool first_image_ = true; // see TODO notes in feature_outliers.cpp

  private:
    cv::Mat sd_frame_;

    // measurement source parameters
    double velocity_floor_;
    double velocity_ceiling_;

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;

    // for destroying windows in tuning mode
    bool drawn_;
  };

}