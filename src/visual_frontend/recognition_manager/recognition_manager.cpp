#include "visual_frontend/recognition_manager/recognition_manager.h"

namespace visual_frontend {

RecognitionManager::RecognitionManager()
{
  // initialize track recognition with default type
  // (the default type is overwritten by param server on first iteration)
  set_method(NONE);
}

// ----------------------------------------------------------------------------

void RecognitionManager::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // check if recognition type has changed
  if (recognition_method_type_ != static_cast<enum RecognitionMethodType>(config.recognition_type))
    set_method(static_cast<enum RecognitionMethodType>(config.recognition_type));

  // if method is not nullptr, update parameters
  if (recognition_method_)
    recognition_method_->set_parameters(config);

  // update crop width and force it to be odd
  crop_width_ = config.crop_width;
  if (!(crop_width_ % 2))
    crop_width_++;
}

// ----------------------------------------------------------------------------

uint32_t RecognitionManager::identify_target(const double x, const double y)
{
  // if no method is selected (NONE), return 0 (no ID recognized)
  if (recognition_method_ == nullptr)
    return 0;

  // get pixel location in the hd frame
  cv::Point center;

  // treat points in the normalized image plane as a 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> center_h; // homogeneous
  std::vector<cv::Point2f> center_d; // distorted

  // populate the vector of the homogeneous point
  center_h.push_back(cv::Point3f(x, y, 1));

  // project the estimate into pixel space
  cv::projectPoints(center_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, center_d);
  center = center_d[0];

  // use pixel location to crop a subimage
  cv::Mat subimage = crop(center);

  // send image to be compared to history and identified
  uint32_t idx = recognition_method_->identify_target(subimage);
  return idx;
}

// ----------------------------------------------------------------------------

void RecognitionManager::update_descriptors(const visual_mtt::TracksPtr& data)
{
  // if no method is selected (NONE), return 0 (no ID recognized)
  if (recognition_method_ == nullptr)
    return;

  // for each published track get a subimage and call update_descriptors
  for (uint32_t i=0; i<data->tracks.size(); i++)
  {
    // for convenience
    visual_mtt::Track track = data->tracks[i];
    double x = track.position.x;
    double y = track.position.y;

    // get pixel location in the hd frame
    cv::Point center;

    // treat points in the normalized image plane as a 3D points (homogeneous).
    // project the points onto the sensor (pixel space) for plotting.
    // use no rotation or translation (world frame = camera frame).
    std::vector<cv::Point3f> center_h; // homogeneous
    std::vector<cv::Point2f> center_d; // distorted

    // populate the vector of the homogeneous point
    center_h.push_back(cv::Point3f(x, y, 1));

    // project the estimate into pixel space
    cv::projectPoints(center_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, center_d);
    center = center_d[0];

    // use pixel location to crop a subimage
    cv::Mat subimage = crop(center);

    // update historical visual information about target
    recognition_method_->update_descriptors(subimage, track.id);
  }
}

// ----------------------------------------------------------------------------

void RecognitionManager::update_image(const cv::Mat hd_frame)
{
  // save the high definition frame
  hd_frame_ = hd_frame;
}

// ----------------------------------------------------------------------------

void RecognitionManager::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  // update local camera parameters (for high definition frame)
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void RecognitionManager::set_method(enum RecognitionMethodType type)
{
  if (type == NONE)
    recognition_method_ = nullptr;
  else if (type == TEMPLATE_MATCHING)
    recognition_method_ = std::make_shared<TemplateMatching>();
  else if (type == BAG_OF_WORDS)
    ROS_WARN("BAG_OF_WORDS not implemented");

  // store the recognition method for later
  recognition_method_type_ = type;
}

// ----------------------------------------------------------------------------

cv::Mat RecognitionManager::crop(cv::Point center)
{
  // x and y indicate the top-left position of the template
  int x = center.x - (crop_width_-1)/2;
  int y = center.y - (crop_width_-1)/2;

  // don't overlap the edges of frame and stay away from edge by 1 pixel
  // TODO: this logic will fail if crop_width_ > hd_frame_ height or width
  if (x < 1) x = 1;
  if (y < 1) y = 1;
  if (x+crop_width_ > hd_frame_.cols-1) x = hd_frame_.cols-crop_width_-1;
  if (y+crop_width_ > hd_frame_.rows-1) y = hd_frame_.rows-crop_width_-1;

  cv::Mat subimage = hd_frame_(cv::Rect(x, y, crop_width_, crop_width_));
  return subimage;
}

}
