#pragma once

#include <iostream>
// #include <opencv2/opencv.hpp>

// dynamic reconfig
// #include "visual_mtt/visual_frontendConfig.h"

// available measurement generation methods
// #include "measurement_source.h"
// #include "feature_outliers.h"

namespace visual_frontend {


  class SourceManager
  {
  public:
    SourceManager();
    // void generate_measurements();
    // void set_parameters(visual_mtt::visual_frontendConfig& config);


  private:
    // vector of sources
    // std::vector<std::shared_ptr<MeasurementSource>> measurement_sources_;

  };

}