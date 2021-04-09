#pragma once

#include <iostream>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// common
#include "common/system.h"

// available recognition methods
#include "recognition_method.h"
#include "template_matching.h"

namespace visual_frontend {

  enum RecognitionMethodType { NONE, TEMPLATE_MATCHING, BAG_OF_WORDS };

  class RecognitionManager
  {
  public:
    RecognitionManager();

    // search among track descriptors (for GMN elevation event callback)
    uint32_t identify_target(const double x, const double y);

    // update image descriptors using current image (for tracks callback)
    void update_descriptors(const std::vector<RR_Model*> tracks);

    // save high definition image (for video callback)
    void update_image(const cv::Mat hd_frame);

    // update relevant parameters (for dynamic reconfigure callback)
    void set_parameters(visual_mtt::visual_frontendConfig& config);

    // set camera parameters for the high definition frame (used once)
    void set_camera(const cv::Mat& K, const cv::Mat& D);

  private:
    // currently selected recognition method
    std::shared_ptr<RecognitionMethod> recognition_method_;
    enum RecognitionMethodType recognition_method_type_;

    // frame and camera parameters
    cv::Mat hd_frame_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;

    // crop parameters
    uint32_t crop_width_;

    // set method
    void set_method(enum RecognitionMethodType type);

    // crop a subimage based on a desired point
    cv::Mat crop(cv::Point center);
  };

}
