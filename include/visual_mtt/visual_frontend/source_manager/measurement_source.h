#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// this defines the base class for all measurement source classes
// a vector of MeasurementSource objects can be used (polymorphism)

namespace visual_frontend {

  class MeasurementSource
  {
  public:

    virtual void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform) = 0;
    virtual void set_parameters(visual_mtt::visual_frontendConfig& config) = 0;

    std::vector<cv::Point2f> features_;
    std::vector<cv::Point2f> features_vel_;

    std::string name_;
  private:

  };

}