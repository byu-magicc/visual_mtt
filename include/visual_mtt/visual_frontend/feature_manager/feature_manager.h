#pragma once

#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// Available Feature Trackers
#include "feature_tracker.h"
#include "lkt_tracker.h"

namespace visual_frontend {

  enum FeatureTrackerType { KLT_TRACKER, ORB_BF_MATCHER };

  class FeatureManager
  {
  public:
    FeatureManager();

    // Using the dynamic reconfigure object created by ROS, update parameters
    void set_parameters(visual_mtt::visual_frontendConfig& config);

    // Give the feature manager the camera matrix and distortion coefficients.
    // Expect all features generated herein to be returned in normalized
    // image coordinates, i.e., `prev_matched_` and `next_matched_`.
    void set_camera(const cv::Mat& K, const cv::Mat& D);

    // This method finds correspondences based on the new frame arguments
    // and the previous frame stored in the FeatureManager's memory.
    // This method should be called once with every new incoming image.
    void find_correspondences(cv::Mat& frame);

    // Expose the matched feature correspondences between the previous
    // frame and this current frame. Updated after a call to `find_correspondences`
    std::vector<cv::Point2f> prev_matched_;
    std::vector<cv::Point2f> next_matched_;

  private:
    // feature detector/tracker
    std::shared_ptr<FeatureTracker> feature_tracker_;
    enum FeatureTrackerType feature_tracker_type_;

    // camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;

    void set_tracker(enum FeatureTrackerType type);
  };

}