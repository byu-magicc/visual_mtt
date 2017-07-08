#include "visual_frontend/homography_manager/homography_manager.h"

namespace visual_frontend {

HomographyManager::HomographyManager()
{
  
  // Initialize feature tracker with default type
  set_method(SIMPLE_HOMOGRAPHY);
}

// ----------------------------------------------------------------------------

void HomographyManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // reprojection_error_ = config.reprojection_error;
}

// ----------------------------------------------------------------------------

void HomographyManager::calculate_homography(const std::vector<cv::Point2f>& prev_features, const std::vector<cv::Point2f>& next_features)
{

  // Find the homography that relates the previous frame to the current frame
  // and expose that homography as a public data member.
  homography_ = homography_method_->get_homography(prev_features, next_features);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void HomographyManager::set_method(enum HomographyMethodType type)
{

  // Create a instance of a private node handle
  ros::NodeHandle nh("~");

  if (type == SIMPLE_HOMOGRAPHY)
  {
    // Create a new homography method
    homography_method_ = std::make_shared<SimpleHomography>();
  }
  else if (type == HOMOGRAPHY_FILTER)
  {
    ROS_WARN("HOMOGRAPHY_FILTER not implemented");
  }

  // Store what type of homography method we are for later
  homography_method_type_ = type;

}

}