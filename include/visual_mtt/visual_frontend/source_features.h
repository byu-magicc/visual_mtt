#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

  class SourceFeatures: public Source
  {
  public:
    SourceFeatures();
    void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform);
    void set_parameters(visual_mtt::visual_frontendConfig& config);

    bool first_image_ = true; // see TODO notes in source_features.cpp

  private:
    double velocity_floor_;
    double velocity_ceiling_;
  };

}