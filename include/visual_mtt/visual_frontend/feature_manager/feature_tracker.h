#pragma once

namespace visual_mtt {

  class FeatureTracker
  {
  public:
    // This method receives an image and it is expected that it returns
    // feature correspondences (i.e., matched features) across the previous
    // image and the currently passed in image.
    virtual void find_correspondences(const cv::Mat& img, std::vector<cv::Point2f>& prev_matched, std::vector<cv::Point2f>& next_matched) = 0;

    virtual void set_max_features(int max_points, int max_) = 0;

    bool first_image_ = true;
  private:
      
  };

}