#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/recognition_manager/recognition_method.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

namespace visual_frontend {

  class TemplateMatching: public RecognitionMethod
  {
  public:
    TemplateMatching();
    ~TemplateMatching();
    // void set_parameters(visual_mtt::visual_frontendConfig& config);
    // void set_camera(const cv::Mat& K, const cv::Mat& D);


  private:
    // cv::Mat sd_frame_;

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;

  };

}
