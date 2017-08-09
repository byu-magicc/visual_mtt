#include "visual_frontend/recognition_manager/recognition_manager.h"

namespace visual_frontend {

RecognitionManager::RecognitionManager()
{

  // Initialize feature tracker with default type
  set_method(TEMPLATE_MATCHING);
}

// ----------------------------------------------------------------------------

void RecognitionManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // check if method has changed
  // if changed, call set_method then call set_parameters, set_camera, and update_image
  // of recognition_method_ using the saved versions here
}

// ----------------------------------------------------------------------------

uint32_t RecognitionManager::find_track_idx(const double x, const double y)
{
  // find the id of the new track (call the set method to do it)
  return (uint32_t)0;
}

// ----------------------------------------------------------------------------

void RecognitionManager::update_descriptors(const visual_mtt::TracksPtr& data)
{
  // use track locations to update the descriptors
  // need to decide if cycling through and cropping would be done here, then
  // sending the cropped part in with the id
  // OR
  // if this would just call the selected version class method and cycle there

  // if all approaches require the exact same cycle+crop approach, then
  // do it here. also, if done here, the camera parameters and frame
  // don't need to be passed all over the place. afterall this is the manager,
  // perhaps such is the roll of the manager, to manage the cropping, etc?
}

// ----------------------------------------------------------------------------

void RecognitionManager::update_image(const cv::Mat hd_frame)
{
  // save the high definition frame
}

// ----------------------------------------------------------------------------

void RecognitionManager::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  // save the camera parameters
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void RecognitionManager::set_method(enum RecognitionMethodType type)
{

  // // Create a instance of a private node handle
  // ros::NodeHandle nh("~");
  //
  // if (type == SIMPLE_HOMOGRAPHY)
  // {
  //   // Create a new homography method
  //   homography_method_ = std::make_shared<SimpleHomography>();
  // }
  // else if (type == HOMOGRAPHY_FILTER)
  // {
  //   ROS_WARN("HOMOGRAPHY_FILTER not implemented");
  // }
  //
  // // Store what type of homography method we are for later
  // homography_method_type_ = type;



}

}
