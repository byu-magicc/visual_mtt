#pragma once

#include <ros/console.h>

#include <opencv2/opencv.hpp>

#include "common/gpu.h"
#include "feature_tracker.h"

namespace visual_frontend {

#ifdef OPENCV_CUDA
  typedef cv::cuda::CornersDetector cvFeatureDetector_t;
#else
  typedef cv::GFTTDetector cvFeatureDetector_t;
#endif

  class LKTTracker : public FeatureTracker
  {
  public:
    LKTTracker(double corner_quality, double corner_quality_min, double corner_quality_max, double corner_quality_alpha, int pyramid_size);

    virtual void find_correspondences(const cv::Mat& img, std::vector<cv::Point2f>& prev_matched, std::vector<cv::Point2f>& next_matched, const cv::Mat& mask);
    virtual void set_max_features(int points_max);

  private:
    std::vector<cv::Point2f> prev_features_;

    // node parameters
    double corner_quality_;         // Corner Quality param of GFFT
    double corner_quality_min_;
    double corner_quality_max_;
    double corner_quality_alpha_;   // Parameter of alpha-filter to adaptively tune corner quality

    // feature generator
    cv::Ptr<cvFeatureDetector_t> gftt_detector_;

    // keep pyramids from the last iteration
    std::vector<cv::Mat> last_pyramids_;  // Primarily for non-CUDA
    cv::Mat last_mono_;                   // Primarily for CUDA
    cv::Size pyramid_size_;

    // Settings
    cv::TermCriteria kltTerm_;

    // Create a feature detector object
    cv::Ptr<cvFeatureDetector_t> init_gftt(int points_max = 0);

    // Wrapper methods to calculate LK optical flow and to detect features
    void calculate_flow(const cv::Mat& mono, std::vector<cv::Point2f>& next_features, std::vector<unsigned char>& valid);
    void detect_features(const cv::Mat& mono, std::vector<cv::Point2f>& features, const cv::Mat& mask);

  };

}