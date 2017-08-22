#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// this defines the base class for all target recognition classes
// a shared pointer of objects is used in the manager

namespace visual_frontend {

  class RecognitionMethod
  {
  public:
    // This receives images of a recently-elevated good models. It is then
    // compared to visual information of historical tracks. It returns an ID
    // number (GMN). A return of 0 means no match was made to previous tracks.

    // use new target image and historical descriptors to identify
    virtual uint32_t identify_target(cv::Mat image) = 0;

    // update the historical descriptors or clustering algorithm
    virtual void update_descriptors(cv::Mat image, uint32_t idx) = 0;

    // update relevant parameters (for dynamic reconfigure callback)
    virtual void set_parameters(visual_mtt::visual_frontendConfig& config) = 0;
  };

}
