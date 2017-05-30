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
  Source(){};
  ~Source(){};

  virtual void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& features, std::vector<cv::Point2f>& features_vel){};
  virtual void set_parameters(visual_mtt2::visual_frontendConfig& config){};

  std::vector<cv::Point2f> features_;
	std::vector<cv::Point2f> features_vel_;

  // how to structure measurements? using an object might be overkill since
  // it will be immediately transformed to a ros message TODO: discussion.

private:
  std::string name;

};