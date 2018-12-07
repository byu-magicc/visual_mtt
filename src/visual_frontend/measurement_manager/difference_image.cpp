#include "visual_frontend/measurement_manager/difference_image.h"

namespace visual_frontend {

DifferenceImage::DifferenceImage()
{
  enabled_ = false;
  name_ = "Difference Image";
  id_ = 1;
  has_velocity_ = false;

  drawn_ = false;
  extra_plots_drawn_ = false;
}

// ----------------------------------------------------------------------------

DifferenceImage::~DifferenceImage()
{
  DestroyWindows(); 
}

// ----------------------------------------------------------------------------

void DifferenceImage::Initialize(const common::Params& params) {}

// ----------------------------------------------------------------------------

void DifferenceImage::SetParameters(const visual_mtt::visual_frontendConfig& config)
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
  if (!extra_plots_)
    DestroyExtraWindows();

  // noise parameters (only for storage, not used in measurement generation)
  sigmaR_pos_ = config.difference_image_sigmaR_pos;
  sigmaR_vel_ = 0;

  size_known_ = false;

  ShouldReset(config.difference_image_enabled);
}

// ----------------------------------------------------------------------------

void DifferenceImage::DrawMeasurements(const common::System& sys)
{
  if (!frame_difference_.empty() && extra_plots_)
  {
    extra_plots_drawn_ = true;

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

  cv::Mat draw = sys.sd_frame_.clone();

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> meas_pos_h; // homogeneous
  std::vector<cv::Point2f> meas_pos_d; // distorted
  if (meas_pos_.size()>0)
  {
    cv::convertPointsToHomogeneous(meas_pos_, meas_pos_h);
    cv::projectPoints(meas_pos_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sys.sd_camera_matrix_, sys.dist_coeff_, meas_pos_d);
  }

  // plot measurements
  for (int j=0; j<meas_pos_d.size(); j++)
  {
    cv::circle(draw, meas_pos_d[j], 3, cv::Scalar(255, 0, 255), 3, CV_AA);
  }

  if (!draw.empty())
  {
    drawn_ = true;
    cv::imshow(name_, draw);
  }

}

// ----------------------------------------------------------------------------

bool DifferenceImage::GenerateMeasurements(const common::System& sys) 
{
  contours1_.clear();
  contours2_.clear();
  meas_pos_.clear();
  // meas_vel_.clear();

  bool good_measurements = false;

  if (!size_known_)
  {
    size_ = sys.sd_frame_.size();
    corners_.clear();
    corners_.push_back(cv::Point2f(border_,             border_));
    corners_.push_back(cv::Point2f(size_.width-border_, border_));
    corners_.push_back(cv::Point2f(size_.width-border_, size_.height-border_));
    corners_.push_back(cv::Point2f(border_,             size_.height-border_));
    size_known_ = true;
  }

  // undisort the low resolution image
  cv::Mat frame_u;
  cv::undistort(sys.sd_frame_, frame_u, sys.sd_camera_matrix_, sys.dist_coeff_);

#if OPENCV_CUDA
  frame_u_.upload(frame_u);
#else
  frame_u_ = frame_u;
#endif

  if (!first_image_ && sys.good_transform_)
  {
    // convert the euclidean transform to pixel transform
    sys.sd_camera_matrix_.convertTo(camera_matrix_, CV_32FC1); // needed for inverse
    cv::Mat transform_pixel = camera_matrix_*sys.transform_*camera_matrix_.inv();

#if OPENCV_CUDA

    // transform previous image using new homography
    cv::cuda::GpuMat frame_u_last_warped;
    cv::cuda::warpPerspective(frame_u_last_, frame_u_last_warped, transform_pixel, size_);

    // raw difference
    cv::cuda::absdiff(frame_u_, frame_u_last_warped, frame_difference_);

    // mask the artifact edges
    MaskEdges(frame_difference_, frame_blur_, transform_pixel);

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
    cv::warpPerspective(frame_u_last_, frame_u_last_, transform_pixel, size_);

    // raw difference
    cv::absdiff(frame_u_, frame_u_last_, frame_difference_);

    // mask the artifact edges
    MaskEdges(frame_difference_, frame_blur_, transform_pixel);

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
        meas_pos_.push_back(centroid);
      }
    }

    // transform points to the normalized image plane
    // for distortion use empty cv::Mat (compensation occurred above)
    if (meas_pos_.size() > 0)
    {
      cv::Mat dist_coeff; // empty by design
      cv::undistortPoints(meas_pos_, meas_pos_, sys.sd_camera_matrix_, dist_coeff);
    }
  }
  else
  {
    first_image_ = false;
  }

  // bump undistorted image (save, overwriting the old one)
  frame_u_last_ = frame_u_.clone();

  if (meas_pos_.size() > 0)
    good_measurements = true;
  else
    ROS_DEBUG("DifferenceImage: No measurements found!");

  return good_measurements;

}

// ----------------------------------------------------------------------------

#if OPENCV_CUDA
void DifferenceImage::MaskEdges(cv::cuda::GpuMat& difference_raw, cv::cuda::GpuMat& difference_masked, cv::Mat& transform)
#else
void DifferenceImage::MaskEdges(cv::Mat& difference_raw, cv::Mat& difference_masked, cv::Mat& transform)
#endif
{
  // see where the old corners land in current frame
  cv::Mat corners_transformed;
  cv::perspectiveTransform(corners_, corners_transformed, transform);
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
//
//                                 Private Methods
//
// ----------------------------------------------------------------------------

void DifferenceImage::Reset() {

  // Destroy opencv windows
  DestroyWindows(); 
  drawn_ = false;
  extra_plots_drawn_ = false;
  first_image_ = true;

}

// ----------------------------------------------------------------------------

void DifferenceImage::DestroyWindows() {

  if(drawn_)
    cv::destroyWindow(name_); 

  DestroyExtraWindows();

}

// ----------------------------------------------------------------------------

void DifferenceImage::DestroyExtraWindows()
{
  if(extra_plots_drawn_) {
    cv::destroyWindow("(1) difference");
    cv::destroyWindow("(2) blur");
    cv::destroyWindow("(3) normalize");
    cv::destroyWindow("(4) threshold");
    cv::destroyWindow("(5) open");
    cv::destroyWindow("(6) all contours");
    cv::destroyWindow("(7) filtered contours");
    extra_plots_drawn_ = false;
  }
}

}


// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::DifferenceImage, visual_frontend::MeasurementBase)
