#pragma once

#include <iostream>
#include <stdexcept>

#include <rransac/tracker.h>

#include "visual_frontend/accessors.h"

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// available measurement generation methods
#include "measurement_source.h"
#include "feature_motion.h"
#include "difference_image.h"

namespace visual_frontend {

  class SourceManager
  {
  public:
    SourceManager();

    // Generates measurements from each measurement source,
    // then feeds the rransac tracker with those measurements.
    void feed_rransac(rransac::Tracker& tracker,
                        cv::Mat& hd_frame, cv::Mat& sd_frame,
                        cv::Mat& homography, bool good_transform, 
                        std::vector<cv::Point2f>& prev_features,
                        std::vector<cv::Point2f>& next_features);

    void set_parameters(visual_mtt::visual_frontendConfig& config);
    void set_camera(const cv::Mat& K, const cv::Mat& D);

    // vector of sources
    std::vector<std::shared_ptr<MeasurementSource>> measurement_sources_;

    // current number of sources
    int n_sources_;

    // whether or not to draw measurements
    bool tuning_;

  private:
    void set_sources();

    // default sources (to be overwritten by .yaml)
    bool feature_motion_    = true;
    bool difference_image_  = false;

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;
  };

}