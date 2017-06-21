#pragma once

#include <ros/console.h>

#include <opencv2/opencv.hpp>

#include "feature_tracker.h"

namespace visual_frontend {

  class LKTTracker : public FeatureTracker
  {
  public:
    LKTTracker(double corner_quality, double corner_quality_min, double corner_quality_max, double corner_quality_alpha, int pyramid_size);
    
    virtual void find_correspondences(const cv::Mat& img, std::vector<cv::Point2f>& prev_matched, std::vector<cv::Point2f>& next_matched);
    virtual void set_max_features(int max_points, int max_);

  private:
    std::vector<cv::Point2f> prev_features_;

    // node parameters
    int    points_target_;          // Target number of points for GFFT to find
    double corner_quality_;         // Corner Quality param of GFFT
    double corner_quality_min_;
    double corner_quality_max_;
    double corner_quality_alpha_;   // Parameter of alpha-filter to adaptively tune corner quality

    // feature generator
    cv::Ptr<cv::GFTTDetector> gftt_detector_;

    // keep pyramids from the last iteration
    std::vector<cv::Mat> last_pyramids_;
    cv::Size pyramid_size_;

    // Settings
    cv::TermCriteria kltTerm_;

    // Extract key points and place in a new vector of cv::Point2f
    void keyPointVecToPoint2f(std::vector<cv::KeyPoint>& keys, std::vector<cv::Point2f>& pts);
    
  };

}