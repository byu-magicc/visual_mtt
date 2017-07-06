#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// Available homography methods
#include "homography_method.h"
#include "simple_homography.h"

namespace visual_frontend {

  enum HomographyMethodType { SIMPLE_HOMOGRAPHY, HOMOGRAPHY_FILTER };

  class HomographyManager
  {
  public:
    HomographyManager();
    void calculate_homography(const std::vector<cv::Point2f>& prev_features,
                              const std::vector<cv::Point2f>& next_features);
    void set_parameters(visual_mtt::visual_frontendConfig& config);

    cv::Mat homography_;
    std::vector<uchar> inlier_mask_;
    bool good_transform_;

  private:
    // currently selected homography method
    std::shared_ptr<HomographyMethod> homography_method_;
    enum HomographyMethodType homography_method_type_;

    void set_method(enum HomographyMethodType type);
  };

}