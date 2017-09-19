#include "visual_frontend/feature_manager/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)

namespace visual_frontend {

FeatureManager::FeatureManager()
{

  // Initialize feature tracker with default type
  set_tracker(KLT_TRACKER);
}

// ----------------------------------------------------------------------------

void FeatureManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{

  // Check to see if feature tracker type has changed
  if (feature_tracker_type_ != static_cast<enum FeatureTrackerType>(config.feature_type))
    set_tracker(static_cast<enum FeatureTrackerType>(config.feature_type));

  feature_tracker_->set_max_features(config.points_max);
}

// ----------------------------------------------------------------------------

void FeatureManager::set_camera(const cv::Mat& K, const cv::Mat& D, cv::Size res)
{
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();

  // define the boundary of the theoretical undistorted image
  int edge_points = 10; // points per edge
  std::vector<cv::Point2f> boundary;
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(i*(res.width/edge_points), 0));
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(res.width, i*(res.height/edge_points)));
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(res.width - i*(res.width/edge_points), res.height));
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(0, res.height - i*(res.height/edge_points)));

  // move points to the normalized image plane (original frame and undistorted
  // frame have the same camera matrix)
  cv::Mat dist_coeff; // we started with the theoretical undistorted image
  cv::undistortPoints(boundary, boundary, camera_matrix_, dist_coeff);

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> boundary_h; // homogeneous
  std::vector<cv::Point2f> boundary_d; // distorted
  cv::convertPointsToHomogeneous(boundary, boundary_h);
  cv::projectPoints(boundary_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, boundary_d);

  // boundary_d is a polygon in the original sd frame, put in matrix form
  cv::Mat boundary_mat(boundary_d);

  // build the mask
  boundary_mat.convertTo(boundary_mat, CV_32SC1);
  mask_ = cv::Mat(res, CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(mask_, boundary_mat, cv::Scalar(255));
}

// ----------------------------------------------------------------------------

void FeatureManager::find_correspondences(cv::Mat& img)
{

  // clear history
  prev_matched_.clear();
  next_matched_.clear();

  // Find the feature correspondences across current and previous frame
  // and expose as public data members: `prev_matched_` and `next_matched_`.
  feature_tracker_->find_correspondences(img, prev_matched_, next_matched_, mask_);

  // compensate for lens distortion and project onto normalized image plane
  if (prev_matched_.size() > 0 && next_matched_.size() > 0)
  {
    cv::undistortPoints(prev_matched_, prev_matched_, camera_matrix_, dist_coeff_);
    cv::undistortPoints(next_matched_, next_matched_, camera_matrix_, dist_coeff_);
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void FeatureManager::set_tracker(enum FeatureTrackerType type)
{

  // Create a instance of a private node handle
  ros::NodeHandle nh("~");

  if (type == KLT_TRACKER)
  {

    // Retrieve the needed parameters for the LKT Tracker
    double cqual, cqual_min, cqual_max, cqual_alpha;
    int pyramid_size;
    nh.param<double>("corner_quality",       cqual,        0.03 );
    nh.param<double>("corner_quality_min",   cqual_min,    0.03 );
    nh.param<double>("corner_quality_max",   cqual_max,    0.05 );
    nh.param<double>("corner_quality_alpha", cqual_alpha,  0.999);
    nh.param<int>   ("pyramid_size",         pyramid_size, 21   );

    // Create a new feature tracker
    feature_tracker_ = std::make_shared<LKTTracker>(cqual, cqual_min, cqual_max, cqual_alpha, pyramid_size);
  }
  else if (type == ORB_BF_MATCHER)
  {
    ROS_WARN("ORB_BF_MATCHER not implemented");
  }

  // Store what type of feature tracker we are for later
  feature_tracker_type_ = type;
}

}