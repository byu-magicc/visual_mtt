#include "visual_frontend/source_manager/difference_image.h"

namespace visual_frontend {

DifferenceImage::DifferenceImage()
{
  name_ = "Difference Image";
}

// ----------------------------------------------------------------------------

DifferenceImage::~DifferenceImage()
{
  cv::destroyWindow(name_);
}

// ----------------------------------------------------------------------------

void DifferenceImage::generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform)
{
  // save the original image for plotting
  sd_frame_ = sd_frame;

  // undisort the low resolution image
  cv::undistort(sd_frame_, frame_u_, camera_matrix_, dist_coeff_);

  if (!first_image_)
  {
    // convert the current euclidean homography to pixel homography
    camera_matrix_.convertTo(camera_matrix_, CV_32FC1); // for inverse
    cv::Mat homography2 = camera_matrix_*homography*camera_matrix_.inv();

    // transform previous image using new homography
    cv::warpPerspective(frame_u_last_, frame_u_last_, homography2, frame_u_.size());

    // difference
    cv::Mat diff;
    cv::absdiff(frame_u_, frame_u_last_, diff);
    cv::imshow("raw difference", diff);

    // morphology

    // grid-based point generation

    // TODO: good_transform flag not yet used! (if false, discard measurements)
    // replace first_image_ logic with good_transform logic and make sure
    // it is set to 'false' in the homography manager on first iteration!
  }
  else
  {
    first_image_ = false;
  }

  // bump undistorted image (save, overwriting the old one)
  frame_u_last_ = frame_u_.clone();

}

// ----------------------------------------------------------------------------

void DifferenceImage::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // example parameter set
  velocity_floor_ = config.filler1;
  velocity_ceiling_ = config.filler2;
}

// ----------------------------------------------------------------------------

void DifferenceImage::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();
}

// ----------------------------------------------------------------------------

void DifferenceImage::draw_measurements()
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
    cv::Scalar color = cv::Scalar(255, 0, 255); // TODO: set before loop, or make member
    cv::circle(draw, features_d[j], 2, color, 2, CV_AA);
  }
  cv::imshow(name_, draw);

  // get the input from the keyboard, force display
  char keyboard = cv::waitKey(1);
  if(keyboard == 'q')
    ros::shutdown();

}

}