#include "visual_frontend/recognition_manager/recognition_manager.h"

namespace visual_frontend {

RecognitionManager::RecognitionManager()
{
  // initialize track recognition with default type
  set_method(NONE);
}
// TODO: alternative constructor with enum passed in?

// ----------------------------------------------------------------------------

void RecognitionManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // check if method has changed
  // if changed, call set_method then set_parameters of recognition_method_

  std::cout << "updating callback parameters" << std::endl;
}

// ----------------------------------------------------------------------------

uint32_t RecognitionManager::identify_target(const double x, const double y)
{
  // find the id of the new track (call the set method to do it)
  return (uint32_t)0;
}

// ----------------------------------------------------------------------------

void RecognitionManager::update_descriptors(const visual_mtt::TracksPtr& data)
{
  // for each published track, extract the local image and call
  // update_descriptors passing in the image and the GMN

  std::cout << "updating all descriptors" << std::endl;
}

// ----------------------------------------------------------------------------

void RecognitionManager::update_image(const cv::Mat hd_frame)
{
  // save the high definition frame

  std::cout << "updating local hd frame" << std::endl;
}

// ----------------------------------------------------------------------------

void RecognitionManager::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  // save the high definition camera parameters

  std::cout << "updating camera parameters" << std::endl;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void RecognitionManager::set_method(enum RecognitionMethodType type)
{
  if (type == NONE)
  {
    recognition_method_ = nullptr;
  }
  else if (type == TEMPLATE_MATCHING)
  {
    // repopulate the recognition method pointer
    recognition_method_ = std::make_shared<TemplateMatching>();
  }
  else if (type == BAG_OF_WORDS)
  {
    ROS_WARN("BAG_OF_WORDS not implemented");
  }

  // store the recognition method for later
  recognition_method_type_ = type;
}

}
