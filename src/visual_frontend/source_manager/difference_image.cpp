#include "visual_frontend/source_manager/difference_image.h"

namespace visual_frontend {

DifferenceImage::DifferenceImage()
{
  name_ = "Difference Image";
  drawn_ = false;
}

// ----------------------------------------------------------------------------

DifferenceImage::~DifferenceImage()
{
  if (drawn_)
  {
    cv::destroyWindow("(1) difference");
    cv::destroyWindow("(2) blur");
    cv::destroyWindow("(3) normalize");
    cv::destroyWindow("(4) threshold");
    cv::destroyWindow("(5) open");
    cv::destroyWindow("(6) all contours");
    cv::destroyWindow("(7) filtered contours");

    cv::destroyWindow(name_); // close last
  }
}

// ----------------------------------------------------------------------------

void DifferenceImage::generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform)
{
  contours1_.clear();
  contours2_.clear();
  features_.clear();

  sd_frame_ = sd_frame;

  if (!size_known_)
  {
    size_ = sd_frame_.size();
    corners_.clear();
    corners_.push_back(cv::Point2f(border_,             border_));
    corners_.push_back(cv::Point2f(size_.width-border_, border_));
    corners_.push_back(cv::Point2f(size_.width-border_, size_.height-border_));
    corners_.push_back(cv::Point2f(border_,             size_.height-border_));
    size_known_ = true;
  }

  // undisort the low resolution image
  cv::Mat frame_u;
  cv::undistort(sd_frame_, frame_u, camera_matrix_, dist_coeff_);

#if OPENCV_CUDA
  frame_u_.upload(frame_u);
#else
  frame_u_ = frame_u;
#endif

  if (!first_image_)
  {
    // convert the euclidean homography to pixel homography
    camera_matrix_.convertTo(camera_matrix_, CV_32FC1); // needed for inverse
    cv::Mat homography_pixel = camera_matrix_*homography*camera_matrix_.inv();

#if OPENCV_CUDA

    // transform previous image using new homography
    cv::cuda::GpuMat frame_u_last_warped;
    cv::cuda::warpPerspective(frame_u_last_, frame_u_last_warped, homography_pixel, size_);

    // raw difference
    cv::cuda::absdiff(frame_u_, frame_u_last_warped, frame_difference_);

    // mask the artifact edges
    mask_edges(frame_difference_, frame_blur_, homography_pixel);

    // blur
    filter_gauss_->apply(frame_blur_, frame_blur_);

    // normalize
    cv::cuda::normalize(frame_blur_, frame_normalized_, 0, 255, cv::NORM_MINMAX, -1);

    // threshold
    cv::cuda::threshold(frame_normalized_, frame_threshold_, threshold_, 255, cv::THRESH_BINARY);

    // open operation (erode then dilate)
    filter_open_->apply(frame_threshold_, frame_open_);
    for (int i=1; i<morph_iterations_; i++)
      filter_open_->apply(frame_open_, frame_open_);

    cv::Mat frame_open(frame_open_);

#else

    // transform previous image using new homography
    cv::warpPerspective(frame_u_last_, frame_u_last_, homography_pixel, size_);

    // raw difference
    cv::absdiff(frame_u_, frame_u_last_, frame_difference_);

    // mask the artifact edges
    mask_edges(frame_difference_, frame_blur_, homography_pixel);

    // blur
    cv::GaussianBlur(frame_blur_, frame_blur_, blur_kernel_, blur_sigma_);

    // normalize
    cv::normalize(frame_blur_, frame_normalized_, 0, 255, cv::NORM_MINMAX);

    // threshold
    cv::threshold(frame_normalized_, frame_threshold_, threshold_, 255, cv::THRESH_BINARY);

    // open operation (erode then dilate)
    cv::morphologyEx(frame_threshold_, frame_open_, cv::MORPH_OPEN, element_, cv::Point(-1,-1), morph_iterations_);

    cv::Mat frame_open = frame_open_;

#endif

    // turn the edges into contours (multiple series of points)
    cv::findContours(frame_open, contours1_, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // filter difference outlines based on contour complexity
    for (int i; i<contours1_.size(); i++)
    {
      int complexity = contours1_[i].size();
      if (complexity > min_complexity_ && complexity < max_complexity_)
      {
        contours2_.push_back(contours1_[i]);

        // convert the std::vector of cv::Point (int) to cv::Mat (float)
        cv::Mat points(contours1_[i]);
        points.convertTo(points, CV_32FC1);

        // get the centroid of the points
        cv::reduce(points, points, 0, CV_REDUCE_AVG);
        cv::Point2f centroid(points.at<float>(0), points.at<float>(1));
        features_.push_back(centroid);
        features_vel_.push_back(cv::Point2f(0,0)); // no velocity provided TODO: fix this hack
      }
    }

    // transform points to the normalized image plane
    // for distortion use empty cv::Mat (compensation occurred above)
    if (features_.size()>0)
    {
      cv::Mat dist_coeff; // empty by design
      cv::undistortPoints(features_, features_, camera_matrix_, dist_coeff);
    }

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

#if OPENCV_CUDA
void DifferenceImage::mask_edges(cv::cuda::GpuMat& difference_raw, cv::cuda::GpuMat& difference_masked, cv::Mat& homography)
#else
void DifferenceImage::mask_edges(cv::Mat& difference_raw, cv::Mat& difference_masked, cv::Mat& homography)
#endif
{
  // see where the old corners land in current frame
  cv::Mat corners_transformed;
  cv::perspectiveTransform(corners_, corners_transformed, homography);
  corners_transformed.convertTo(corners_transformed, CV_32SC1);

  // generate mask
  cv::Mat mask(size_, CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(mask, corners_transformed, cv::Scalar(255));

  // apply mask
#if OPENCV_CUDA
  cv::cuda::cvtColor(difference_raw, difference_masked, CV_BGR2GRAY);
  cv::cuda::bitwise_and(difference_masked, mask, difference_masked);
#else
  cv::cvtColor(difference_raw, difference_masked, CV_BGR2GRAY);
  cv::bitwise_and(difference_masked, mask, difference_masked);
#endif
}

// ----------------------------------------------------------------------------

void DifferenceImage::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // blur parameters
  blur_kernel_ = cv::Size(2*config.blur_kernel + 1, 2*config.blur_kernel + 1);
  blur_sigma_ = config.blur_sigma;
#if OPENCV_CUDA
  filter_gauss_ = cv::cuda::createGaussianFilter(0, 0, blur_kernel_, blur_sigma_);
#endif

  // morphology open parameters
  element_ = cv::getStructuringElement(
    cv::MORPH_RECT,
    cv::Size(2*config.morph_size + 1, 2*config.morph_size+1),
    cv::Point(config.morph_size, config.morph_size));
  morph_iterations_ = config.morph_iterations;
#if OPENCV_CUDA
  filter_open_ = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, 0, element_);
#endif

  // other key parameters
  threshold_ = config.threshold;
  min_complexity_ = config.min_complexity;
  max_complexity_ = config.max_complexity;

  extra_plots_ = config.difference_extra_plots;
  if (!extra_plots_ && drawn_)
  {
    cv::destroyWindow("(1) difference");
    cv::destroyWindow("(2) blur");
    cv::destroyWindow("(3) normalize");
    cv::destroyWindow("(4) threshold");
    cv::destroyWindow("(5) open");
    cv::destroyWindow("(6) all contours");
    cv::destroyWindow("(7) filtered contours");
  }

  size_known_ = false;
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
  if (!frame_difference_.empty() && extra_plots_)
  {
    drawn_ = true;
#if OPENCV_CUDA
    cv::Mat frame_difference(frame_difference_);
    cv::Mat frame_blur(frame_blur_);
    cv::Mat frame_normalized(frame_normalized_);
    cv::Mat frame_threshold(frame_threshold_);
    cv::Mat frame_open(frame_open_);
    cv::imshow("(1) difference", frame_difference);
    cv::imshow("(2) blur",       frame_blur);
    cv::imshow("(3) normalize",  frame_normalized);
    cv::imshow("(4) threshold",  frame_threshold);
    cv::imshow("(5) open",       frame_open);
#else
    cv::imshow("(1) difference", frame_difference_);
    cv::imshow("(2) blur",       frame_blur_);
    cv::imshow("(3) normalize",  frame_normalized_);
    cv::imshow("(4) threshold",  frame_threshold_);
    cv::imshow("(5) open",       frame_open_);
#endif

    frame_contours_ = cv::Mat::zeros(size_, CV_64FC3);
    cv::drawContours(frame_contours_, contours1_, -1, cv::Scalar(0,255,0));
    cv::imshow("(6) all contours", frame_contours_);

    frame_points_ = cv::Mat::zeros(size_, CV_64FC3);
    cv::drawContours(frame_points_, contours2_, -1, cv::Scalar(255,0,255));
    cv::imshow("(7) filtered contours", frame_points_);
  }

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
    cv::circle(draw, features_d[j], 3, cv::Scalar(255, 0, 255), 3, CV_AA);
  }

  if (!draw.empty())
  {
    drawn_ = true;
    cv::imshow(name_, draw);
  }

}

}