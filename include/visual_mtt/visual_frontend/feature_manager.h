#include <iostream>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt2/visual_frontendConfig.h"

class FeatureManager
{
public:
  FeatureManager(ros::NodeHandle nh, int nominal_corner_count = 1200, int pyr_size_param = 21);

  void set_parameters(visual_mtt2::visual_frontendConfig& config);
  void find_correspondences(cv::Mat& frame);
  static void keyPointVecToPoint2f(std::vector<cv::KeyPoint>& keys, std::vector<cv::Point2f>& pts);

  std::vector<cv::Point2f> prev_matched_;
  std::vector<cv::Point2f> next_matched_;

private:

  bool first_image_;
  std::vector<cv::Point2f> prev_features_;

  //cv::Mat last_image_; // not used

  // Node parameters
  int max_points_tracked_;
  double corner_quality_;
  double corner_quality_min_;
  double corner_quality_max_;
  double corner_quality_alpha_;
  int nominal_corner_count_;

#if CV_MAJOR_VERSION == 2
  // we won't be supporting opencv2
  cv::Ptr<cv::GoodFeaturesToTrackDetector> gftt_detector;
#elif CV_MAJOR_VERSION == 3
  cv::Ptr<cv::GFTTDetector> gftt_detector_;
#endif

  // keep pyramids from the last iteration
  std::vector<cv::Mat> last_pyramids_;
  cv::Size pyr_size_;
  int pyr_size_param_;

  // Settings
  cv::TermCriteria kltTerm_;

};

