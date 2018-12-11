#include "common/system.h"

namespace common {

System::System()
{
  // Init flags
  ClearFlags();

  resize_scale_ = 1;
  num_of_measurements_ = 0;
  cam_info_received_ = false;
  tuning_ = false;

}

// --------------------------------------------------------------------------------------

System::~System() {}

// --------------------------------------------------------------------------------------

void System::SetHDFrame(const cv::Mat& hd_frame) 
{
	hd_frame_ = hd_frame;
}

// --------------------------------------------------------------------------------------

void System::SetResizeScale(const double resize_scale)
{
	resize_scale_ = resize_scale;
}

// --------------------------------------------------------------------------------------

void System::SetSDFrame()
{
  // resize frame
  if (resize_scale_ != 1)
  {
  #if OPENCV_CUDA
    cv::cuda::GpuMat hd_frame_cuda, sd_frame_cuda;
    hd_frame_cuda.upload(hd_frame_);
    cv::cuda::resize(hd_frame_cuda, sd_frame_cuda, sd_res_, 0, 0, cv::INTER_AREA);
    sd_frame_cuda.download(sd_frame_);
  #else
    cv::resize(hd_frame_, sd_frame_, sd_res_, 0, 0, cv::INTER_AREA);
  #endif
  }
  else {
    sd_frame_ = hd_frame_.clone();
  }

}

// --------------------------------------------------------------------------------------

void System::SetCameraParams(const cv::Mat& K, const cv::Mat& D)
{
	hd_camera_matrix_ = K;
	dist_coeff_ = D;
  cam_info_received_ = true;
}

// --------------------------------------------------------------------------------------

void System::SetScaledCameraParams()
{

  if (cam_info_received_)
  {

    // scale the entire matrix except the 3,3 element
    sd_camera_matrix_ = hd_camera_matrix_ * resize_scale_;
    sd_camera_matrix_.at<double>(2,2) = 1;
    sd_res_.width = hd_frame_.size().width*resize_scale_;
    sd_res_.height = hd_frame_.size().height*resize_scale_;

    // Sets the new undistorted region mask according to the
    // new size of sd_frame_.
    SetUndistortedRegionMask();

  }

}

// --------------------------------------------------------------------------------------

void System::SetUndistortedRegionMask()
{

  // define the boundary of the theoretical undistorted image
  int edge_points = 10; // points per edge
  std::vector<cv::Point2f> boundary;
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(i*(sd_res_.width/edge_points), 0));
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(sd_res_.width, i*(sd_res_.height/edge_points)));
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(sd_res_.width - i*(sd_res_.width/edge_points), sd_res_.height));
  for (uint32_t i=0; i<edge_points; i++)
    boundary.push_back(cv::Point2f(0, sd_res_.height - i*(sd_res_.height/edge_points)));

  // move points to the normalized image plane (original frame and undistorted
  // frame have the same camera matrix)
  cv::Mat dist_coeff; // we started with the theoretical undistorted image
  cv::undistortPoints(boundary, boundary, sd_camera_matrix_,cv::Mat());

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> boundary_h; // homogeneous
  std::vector<cv::Point2f> boundary_d; // distorted
  cv::convertPointsToHomogeneous(boundary, boundary_h);
  cv::projectPoints(boundary_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sd_camera_matrix_, dist_coeff_, boundary_d);
  // cv::projectPoints(boundary_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), cv::Mat::eye(3,3,CV_64FC1), dist_coeff_, boundary_d);

  // boundary_d is a polygon in the original sd frame, put in matrix form
  cv::Mat boundary_mat(boundary_d);

  // build the mask
  boundary_mat.convertTo(boundary_mat, CV_32SC1);
  undistorted_region_mask_ = cv::Mat(sd_res_, CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(undistorted_region_mask_, boundary_mat, cv::Scalar(255));

  // std::cout << "undistorted region mask" << std::endl;
  // std::cout << "sd res: " << sd_res_ << std::endl << undistorted_region_mask_ << std::endl;
  // std::cout << "distortion mask: " << dist_coeff_ << std::endl;
}

// --------------------------------------------------------------------------------------

void System::ClearFlags()
{
  good_features_ = false;
  good_transform_ = false;
  good_measurements_ = false;
}

// --------------------------------------------------------------------------------------

void System::ClearMatchedFeatures()
{
  d_prev_matched_.clear();
  d_curr_matched_.clear();  
  ud_prev_matched_.clear();
  ud_curr_matched_.clear();
}

// --------------------------------------------------------------------------------------

void System::AddMatchedFeatures(const std::vector<cv::Point2f>& d_prev_matched, const std::vector<cv::Point2f>& d_curr_matched)
{

  d_prev_matched_.insert(std::end(d_prev_matched_),
                         std::begin(d_prev_matched),
                         std::end(d_prev_matched));
  d_curr_matched_.insert(std::end(d_curr_matched_),
                         std::begin(d_curr_matched),
                         std::end(d_curr_matched));

}

// --------------------------------------------------------------------------------------

void System::UndistortMatchedFeatures()
{
    // compensate for lens distortion and project onto normalized image plane
  if (d_prev_matched_.size() > 0 && d_curr_matched_.size() > 0)
  {
    cv::undistortPoints(d_prev_matched_, ud_prev_matched_, sd_camera_matrix_, dist_coeff_);
    cv::undistortPoints(d_curr_matched_, ud_curr_matched_, sd_camera_matrix_, dist_coeff_);
  }
}



// --------------------------------------------------------------------------------------

void System::SetFeatureFlag(const bool& good_features)
{
  good_features_ = good_features;
}

// --------------------------------------------------------------------------------------

void System::ClearTransform()
{
  transform_ = cv::Mat::eye(3, 3, CV_32F);
}

// --------------------------------------------------------------------------------------

void System::SetTransform(const cv::Mat& transform)
{
  transform_ = transform;
}

// --------------------------------------------------------------------------------------

void System::SetTransformFlag(const bool& good_transform)
{
  good_transform_ = good_transform;
}

// --------------------------------------------------------------------------------------

void System::ClearMeasurements()
{
  measurements_.clear();
  num_of_measurements_ = 0;
}

// --------------------------------------------------------------------------------------

void System::AddMeasurements(const int id,
                             const bool has_velocity, 
                             const std::vector<cv::Point2f>& meas_pos, 
                             const std::vector<cv::Point2f>& meas_vel)
{

  Measurements measurement;
  measurement.id = id;
  measurement.has_velocity = has_velocity;
  measurement.meas_pos = meas_pos;
  measurement.meas_vel = meas_vel;
  measurement.num_of_measurements = meas_pos.size();;

  measurements_.push_back(measurement);
  num_of_measurements_ += meas_pos.size();

}

// --------------------------------------------------------------------------------------

void System::SetMeasurementFlag(const bool& good_measurements)
{
  good_measurements_ = good_measurements;
}

// --------------------------------------------------------------------------------------


}