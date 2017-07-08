#include "visual_frontend/homography_manager/simple_homography.h"

namespace visual_frontend {

SimpleHomography::SimpleHomography()
{

}

// ----------------------------------------------------------------------------

cv::Mat SimpleHomography::get_homography(const std::vector<cv::Point2f>& prev_features, const std::vector<cv::Point2f>& next_features)
{
  cv::Mat homography;

  // TODO: we should force a higher threshold since they could be bad measurements
  // If there aren't enough feature pairs to find a homography, just bail and let
  // other people know there isn't a transform to be trusted.
  if (prev_features.size() < 4)
  {
    good_transform_ = false;
    homography = cv::Mat::eye(3, 3, CV_32F);
    return homography;
  }

  // calculate the homography
  const double reprojection_error = 0.001; // the optimization aims to minimize below this value
  homography = cv::findHomography(prev_features, next_features, CV_RANSAC, reprojection_error, inlier_mask_);

  // baptize the homography
  homography.convertTo(homography, CV_32F);

  // use inlier count to determine if the homography is good
  int inlier_count = 0;
  for(int i=0; i<inlier_mask_.size(); i++)
    inlier_count += inlier_mask_[i] ? 1 : 0;

  // TODO: should this (20) be a % of the pairs rather than a set value?
  if (inlier_count < 20)
  {
    ROS_WARN_STREAM("(" << "#" << ") " << "homography calculator: few homography inliers (" << inlier_count << ")");
    // TODO: replace # with frame number
    good_transform_ = false;
  }
  else
  {
    good_transform_ = true;
  }

  return homography;
}

// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

}