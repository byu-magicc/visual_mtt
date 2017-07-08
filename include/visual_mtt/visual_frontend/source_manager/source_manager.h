#pragma once

#include <iostream>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// messages
#include "visual_mtt/RRANSACScan.h"

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

    // scan of measurements
    visual_mtt::RRANSACScan scan_;

    // number of sources activated TODO: perhaps not needed for anything
    int n_sources_;

    // whether or not to draw measurements
    bool tuning_;

  private:
    void set_sources();

    // default sources (to be overwritten by .yaml)
    bool feature_motion_    = true;
    bool difference_image_  = false;

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;
  };

}