#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt2/visual_frontendConfig.h"

// this defines the base class for all measurement source classes
// a vector of Source objects can be used (polymorphism)

class Source
{
public:

  virtual void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& features, std::vector<cv::Point2f>& features_vel, bool good_transform) = 0;
  virtual void set_parameters(visual_mtt2::visual_frontendConfig& config) = 0;

  std::vector<cv::Point2f> features_;
  std::vector<cv::Point2f> features_vel_;

  std::string name_;
private:

};