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
    void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& features, std::vector<cv::Point2f>& features_vel, bool good_transform);
    void set_parameters(visual_mtt::visual_frontendConfig& config);

    bool first_image_ = true;

  private:

    // needs cleanup! (some aren't being used)
    // also, any of these used by dynamic reconfigure will be overwritten
    // at launch - consider not assigning values for such.
    //bool SHOW_OPENCV_VIEW_detectMovingPoints = 1;
    //bool SHOW_OPENCV_VIEW_MOTION_MASK = 1;
    //double MIN_FEATURE_DISTANCE  =1.;
    //double MASK_THOLD_  =25;
    //int MORPH_CLOSE_ITERATIONS_ = 1;
    //int MORPH_KERNEL_SIZE_ = 2;
    //double GAUSSIAN_BLUR_SIZE = 5;
    //double GAUSSIAN_BLUR_SIGMA = 2;
    double velocity_floor_;
    double velocity_ceiling_;
    double homography_error_thold_ = 0.25;



  };

}