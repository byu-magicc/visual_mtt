#include "visual_frontend/source_manager/feature_outliers.h"

namespace visual_frontend {

FeatureOutliers::FeatureOutliers()
{
  name_ = "Homography Outliers";
  drawn_ = false;
}

// ----------------------------------------------------------------------------

FeatureOutliers::~FeatureOutliers()
{
  if (drawn_)
  {
    cv::destroyWindow(name_);
  }
}

// ----------------------------------------------------------------------------

void FeatureOutliers::generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform)
{
  sd_frame_ = sd_frame;

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
    for (int ii = 0; ii < next_features.size(); ++ii)
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

void FeatureOutliers::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  velocity_floor_ = config.minimum_pixel_velocity;
  velocity_ceiling_ = config.maximum_pixel_velocity;
}

// ----------------------------------------------------------------------------

void FeatureOutliers::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();
}

// ----------------------------------------------------------------------------

void FeatureOutliers::draw_measurements()
{
  cv::Mat draw = sd_frame_.clone();

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> features_h; // homogeneous
  std::vector<cv::Point2f> features_d; // distorted
  if (features_.size()>0)
  {
    cv::convertPointsToHomogeneous(features_, features_h);
    cv::projectPoints(features_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, features_d);
  }

  // plot measurements
  for (int j=0; j<features_d.size(); j++)
  {
    cv::circle(draw, features_d[j], 2, cv::Scalar(255, 0, 255), 2, CV_AA);
  }

  if (!draw.empty())
  {
    drawn_ = true;
    cv::imshow(name_, draw);
  }

}

}