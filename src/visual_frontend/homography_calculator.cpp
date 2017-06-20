#include "visual_frontend/homography_calculator.h"

// GOAL:
// - provide a modular homography calculator that uses feature correspondences
// - this will be replaced by a homography filter when IMU data is used

namespace visual_frontend {

HomographyCalculator::HomographyCalculator()
{
  // this will probably merge with the homography_filter class eventually
  // into a plain "homography" class, we'll see.
}

// ----------------------------------------------------------------------------

void HomographyCalculator::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  reprojection_error_ = config.reprojection_error;
}

// ----------------------------------------------------------------------------

void HomographyCalculator::calculate_homography(const std::vector<cv::Point2f>& prev_features,
                                                const std::vector<cv::Point2f>& next_features)
{
  // use feature correspondences to find homography

  // TODO: 5 or more are needed for a homography,
  // we should force a higher threshold so there can be outliers (measurements)
  if (prev_features.size() > 4)
  {
    // calculate the homography
    homography_ = cv::findHomography(prev_features, next_features,
      CV_RANSAC, reprojection_error_, inlier_mask_);

    // baptize the homography
    homography_.convertTo(homography_, CV_32F);

    // use inlier count to determine if the homography is good
    int inlier_count = 0;
    for(int i = 0; i < inlier_mask_.size(); ++i)
    {
      inlier_count += inlier_mask_[i] ? 1 : 0;
    }

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
      cv::perspectiveTransform(prev_features, corrected_pts, homography_);

    // Find the point velocities
    // TODO these velocities are in pixels per frame: make this pixels per second.
    pixel_diff_.clear();
    for (int i = 0; i < corrected_pts.size(); ++i)
      pixel_diff_.push_back(next_features[i] - corrected_pts[i]);
  }
  else
  {
    // TODO:
    // the number of feature correspondences was too low to create a homography
    // what to do about:
    // homography_
    // pixel_diff_
    // ?

    // for homography_, we could not update, set "good_transform_" to false
    // for pixel_diff_, we could just clear it
    // then the good_transform_ flag would signal to the measurement methods
    // not to generate new measurements

    // the old homography_ would then be sent to R-RANSAC to "propagate"
    // the histories appropriately. This appraoch would probably only be
    // reliable for a single frame since it isn't a true propagation. The
    // (future) homograpy filter be a much better approach.

    pixel_diff_.clear();
    good_transform_ = false;

    // TODO: create a warning at this else (meaning too few features to generate a homography)
    // TODO: create an error (red) warning if good_transform_=false many times in a row
  }
}

}