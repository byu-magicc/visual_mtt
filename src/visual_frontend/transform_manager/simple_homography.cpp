#include "visual_frontend/transform_manager/simple_homography.h"

namespace visual_frontend {

// ----------------------------------------------------------------------------

SimpleHomography::SimpleHomography()
{
  enabled_ = false;
  drawn_ = false;
  name_ = "Simple Homography";
}

// ----------------------------------------------------------------------------

SimpleHomography::~SimpleHomography(){

  DestroyWindows();
}

// ----------------------------------------------------------------------------

void SimpleHomography::Initialize(const common::Params& params){}

// ----------------------------------------------------------------------------

void SimpleHomography::SetParameters(const visual_mtt::visual_frontendConfig& config) 
{
  ShouldReset(config.simple_homography_enabled);
}

// ----------------------------------------------------------------------------

void SimpleHomography::DrawTransform(const common::System& sys){

  cv::Size frame_size = sys.sd_frame_.size();

  // undistort the low resolution image
  cv::Mat frame_u;
  cv::undistort(sys.sd_frame_, frame_u, sys.sd_camera_matrix_, sys.dist_coeff_);

#if OPENCV_CUDA
  frame_u_.upload(frame_u);
#else
  frame_u_ = frame_u;
#endif


  if (!frame_u_last_.empty())
  {
    cv::Mat camera_matrix;

    // convert the euclidean homography to pixel homography
    sys.sd_camera_matrix_.convertTo(camera_matrix, CV_32FC1); // needed for inverse
    cv::Mat transform_pixel = camera_matrix*transform_*camera_matrix.inv();

  #if OPENCV_CUDA

    // transform previous image using new homography
    cv::cuda::GpuMat frame_u_last_warped;
    cv::cuda::warpPerspective(frame_u_last_, frame_u_last_warped, transform_pixel, frame_size);

    // raw difference
    cv::cuda::absdiff(frame_u_, frame_u_last_warped, frame_difference_);

  #else

    // transform previous image using new homography
    cv::warpPerspective(frame_u_last_, frame_u_last_, transform_pixel, frame_size);

    // raw difference
    cv::absdiff(frame_u_, frame_u_last_, frame_difference_);

  #endif

  #if OPENCV_CUDA
    cv::Mat frame_difference(frame_difference_);
    cv::imshow(name_, frame_difference);
  #else
    cv::imshow(name_, frame_difference_);
  #endif
  }

  drawn_ = true;

  // bump undistorted image (save, overwriting the old one)
  frame_u_last_ = frame_u_.clone();


}

// ----------------------------------------------------------------------------

bool SimpleHomography::GetTransform(const common::System& sys)
{
  good_transform_ = false;

  // If there aren't enough feature pairs to find a homography, just bail and let
  // other people know there isn't a transform to be trusted.
  if (!sys.good_features_)
  {
    transform_ = cv::Mat::eye(3, 3, CV_32F);
  } else {

    // calculate the homography
    const double reprojection_error = 0.001; // the optimization aims to minimize below this value
    transform_ = cv::findHomography(sys.ud_prev_matched_, sys.ud_curr_matched_, CV_RANSAC, reprojection_error, inlier_mask_);

    // check that the homography is not empty
    if (transform_.empty())
    {
      transform_ = cv::Mat::eye(3, 3, CV_32F);
    }

    // baptize the homography
    transform_.convertTo(transform_, CV_32F);

    // use inlier count to determine if the homography is good
    int inlier_count = 0;
    for(int i=0; i<inlier_mask_.size(); i++)
      inlier_count += inlier_mask_[i] ? 1 : 0;

    // TODO: should this (20) be a % of the pairs rather than a set value?
    if (inlier_count < 20)
    {
      ROS_WARN_STREAM_THROTTLE(sys.message_output_period_,"(" << "#" << ") " << "homography calculator: few homography inliers (" << inlier_count << ")");
      // TODO: replace # with frame number
    }
    else
    {
      good_transform_ = true;
    }
  }

  return good_transform_;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void SimpleHomography::Reset() 
{

  DestroyWindows();
  drawn_ = false;

}

void SimpleHomography::DestroyWindows() 
{

  if(drawn_)
    cv::destroyWindow(name_);
}
}

// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::SimpleHomography, visual_frontend::TransformBase)