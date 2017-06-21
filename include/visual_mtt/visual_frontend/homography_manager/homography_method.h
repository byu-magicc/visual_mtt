#pragma once

namespace visual_frontend {

  class HomographyMethod
  {
  public:
    // This method takes in a set of matched features across the current frame
    // and the previous frame. Each index of `prev_features` correspondes to the
    // same index of `next_features`, which is a feature in the current frame.
    // Based on these features (and potentially other things) return the homography
    // between the previous and current frame as type CV_32F.
    virtual cv::Mat get_homography(const std::vector<cv::Point2f>& prev_features, const std::vector<cv::Point2f>& next_features) = 0;

  };

}