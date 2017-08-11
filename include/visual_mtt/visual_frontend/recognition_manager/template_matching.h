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
    // ~TemplateMatching();

    virtual uint32_t identify_target(cv::Mat image);

    virtual void update_descriptors(cv::Mat image, uint32_t idx);


  private:
    // cv::Mat sd_frame_;

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;

  };

}
