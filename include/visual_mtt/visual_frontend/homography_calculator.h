#include <iostream>
#include <opencv2/opencv.hpp>

// dynamic reconfig
#include "visual_mtt2/visual_frontendConfig.h"

class HomographyCalculator
{
public:
  HomographyCalculator();
  void calculate_homography(const std::vector<cv::Point2f>& prev_features,
                            const std::vector<cv::Point2f>& next_features);
  void set_parameters(visual_mtt2::visual_frontendConfig& config);
  cv::Mat homography_;
  std::vector<uchar> inlier_mask_;
  std::vector<cv::Point2f> pixel_diff_;

private:

  double reprojection_error_;

};

