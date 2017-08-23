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

    virtual uint32_t identify_target(cv::Mat image);

    virtual void update_descriptors(cv::Mat image, uint32_t idx);

    virtual void set_parameters(visual_mtt::visual_frontendConfig& config);


  private:
    double thresh1;
    double thresh2;

  };

}
