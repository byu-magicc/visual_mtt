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
    pixel_diff_.clear();
    good_transform_ = false;
    return homography;
  }

  // calculate the homography
  homography = cv::findHomography(prev_features, next_features, CV_RANSAC, reprojection_error_, inlier_mask_);

  // baptize the homography
  homography.convertTo(homography, CV_32F);

  // use inlier count to determine if the homography is good
  int inlier_count = 0;
  for(int i = 0; i < inlier_mask_.size(); ++i)
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

  // Use the homography to transform the points forwards. This will put all of the
  // previous feature points on top of the next feature points except for points on
  // moving objects. Those will be off by some pixels, the pixel velocity.
  std::vector<cv::Point2f> corrected_pts;
  if (prev_features.size() > 0)
    cv::perspectiveTransform(prev_features, corrected_pts, homography);

  // Find the point velocities
  // TODO these velocities are in pixels per frame: make this pixels per second.
  pixel_diff_.clear();
  for (int i = 0; i < corrected_pts.size(); ++i)
    pixel_diff_.push_back(next_features[i] - corrected_pts[i]);

  return homography;
}

// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

}