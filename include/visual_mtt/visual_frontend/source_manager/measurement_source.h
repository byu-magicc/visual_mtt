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
    virtual ~MeasurementSource() {};
    virtual void generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame,
                              cv::Mat& homography, bool good_transform,
                              std::vector<cv::Point2f>& prev_features,
                              std::vector<cv::Point2f>& next_features) = 0;
    
    virtual void set_parameters(visual_mtt::visual_frontendConfig& config) = 0;
    virtual void set_camera(const cv::Mat& K, const cv::Mat& D) = 0;
    virtual void draw_measurements() = 0;

    std::vector<cv::Point2f> features_;
    std::vector<cv::Point2f> features_vel_;

    int id_;
    std::string name_;
    bool has_velocity_;
    double sigmaR_pos_;
    double sigmaR_vel_;
  private:

  };

}