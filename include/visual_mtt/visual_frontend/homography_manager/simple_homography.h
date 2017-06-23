#pragma once

#include <ros/console.h>

#include <opencv2/opencv.hpp>

#include "homography_method.h"

namespace visual_frontend {

  class SimpleHomography : public HomographyMethod
  {
  public:
    SimpleHomography();

    virtual cv::Mat get_homography(const std::vector<cv::Point2f>& prev_features, const std::vector<cv::Point2f>& next_features);


  private:
    std::vector<uchar> inlier_mask_;
    std::vector<cv::Point2f> pixel_diff_;
    double reprojection_error_ = 3;
    bool good_transform_;
    
  };

}