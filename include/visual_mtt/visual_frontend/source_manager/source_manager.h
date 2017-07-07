#pragma once

#include <iostream>
// #include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// available measurement generation methods
#include "measurement_source.h"
#include "feature_outliers.h"

namespace visual_frontend {

  class SourceManager
  {
  public:
    SourceManager();
    void generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform);
    void set_parameters(visual_mtt::visual_frontendConfig& config);
    void set_camera(const cv::Mat& K, const cv::Mat& D);

    // vector of sources
    std::vector<std::unique_ptr<MeasurementSource>> measurement_sources_;

    // number of sources activated
    int n_sources_;

    // whether or not to draw measurements
    bool tuning_;

  private:
    void set_sources();

    // default sources (to be overwritten by .yaml)
    bool feature_motion_    = true;
    bool difference_image_  = false;
    bool dnn_activations_   = false;
  };

}