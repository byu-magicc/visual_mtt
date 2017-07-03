#include "visual_frontend/source_features.h"

namespace visual_frontend {

SourceFeatures::SourceFeatures()
{

  name_ = "Homography Outliers";

}

// ----------------------------------------------------------------------------

void SourceFeatures::generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform)
{
  if (!first_image_)
  {
    // Warp previous features forwards. All static features from sequential
    // frames will be aligned. Features from moving objects will be offset by
    // some distance, a velocity ("normalized image units" per frame).
    std::vector<cv::Point2f> corrected_pts;
    if (prev_features.size() > 0)
      cv::perspectiveTransform(prev_features, corrected_pts, homography);

    // Find the point velocities
    std::vector<cv::Point2f> features_vel;
    for (int i = 0; i < corrected_pts.size(); ++i)
      features_vel.push_back(next_features[i] - corrected_pts[i]);

    // Save points whose disparity exceed the velocity threshold
    features_.clear();
    features_vel_.clear();
    int numberOfPossibleMovers = 0;
    for (int ii = 0; ii < std::min((int)next_features.size(), 200); ++ii)
    {
      float vel = sqrt(features_vel[ii].x*features_vel[ii].x + features_vel[ii].y*features_vel[ii].y);
      if (vel > velocity_floor_ && vel < velocity_ceiling_)
      {
        numberOfPossibleMovers++;
        features_.push_back(next_features[ii]);
        features_vel_.push_back(features_vel[ii]);
      }
    }

    // TODO: good_transform flag not yet used! (if false, discard measurements)
    // replace first_image_ logic with good_transform logic and make sure
    // it is set to 'false' in the homography manager on first iteration!
  }
  else
  {
    first_image_ = false;
  }
}

// ----------------------------------------------------------------------------

void SourceFeatures::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  velocity_floor_ = config.minimum_pixel_velocity;
  velocity_ceiling_ = config.maximum_pixel_velocity;
}

}
