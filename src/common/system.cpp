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

  for(int i = 0; i < num_frame_types_; i++)
  {
    frames_required_[i] = false;
  }

  for(int i = 0; i < num_cuda_frame_types_; i++)
  {
    cuda_frames_required_[i] = false;
  }

}

// --------------------------------------------------------------------------------------

System::~System() {}

// --------------------------------------------------------------------------------------

void System::SetHDFrame(const cv::Mat& hd_frame) 
{
	hd_frame_ = hd_frame;
  frame_exists_[HD] = true;
}

// --------------------------------------------------------------------------------------

void System::SetResizeScale(const double resize_scale)
{
	resize_scale_ = resize_scale;
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

#if OPENCV_CUDA
  cv::Mat map1, map2;
  cv::initUndistortRectifyMap(sd_camera_matrix_, dist_coeff_,
                              cv::Mat(), sd_camera_matrix_, sd_res_,
                              CV_32F, map1, map2);
  undistort_map_x_.upload(map1);
  undistort_map_y_.upload(map2);
#endif
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

cv::Mat System::GetFrame(frame_type_ frame_type) const
{
  switch(frame_type)
  {
    case HD:
      if(hd_frame_.empty())
      {
        std::cerr << "ERROR: HD frame has not been set" << std::endl;
      }
      return hd_frame_;
    case SD:
      if(sd_frame_.empty())
      {
        std::cerr << "ERROR: SD frame has not been set" << std::endl;
      }
      return sd_frame_;
    case MONO:
      if(mono_frame_.empty())
      {
        std::cerr << "ERROR: Mono frame has not been set" << std::endl;
      }
      return mono_frame_;
    case UNDIST:
      if(undist_frame_.empty())
      {
        std::cerr << "ERROR: Undistorted frame has not been set" << std::endl;
      }
      return undist_frame_;
    case HSV:
      if(hsv_frame_.empty())
      {
        std::cerr << "ERROR: HSV frame has not been set" << std::endl;
      }
      return hsv_frame_;
  }
}

cv::cuda::GpuMat System::GetCUDAFrame(frame_type_cuda_ frame_type_cuda) const
{
  switch(frame_type_cuda)
  {
    case HD_CUDA:
      if(hd_frame_cuda_.empty())
      {
        std::cerr << "ERROR: HD CUDA frame has not been set" << std::endl;
      }
      return hd_frame_cuda_;
    case SD_CUDA:
      if(sd_frame_cuda_.empty())
      {
        std::cerr << "ERROR: SD CUDA frame has not been set" << std::endl;
      }
      return sd_frame_cuda_;
    case MONO_CUDA:
      if(mono_frame_cuda_.empty())
      {
        std::cerr << "ERROR: Mono CUDA frame has not been set" << std::endl;
      }
      return mono_frame_cuda_;
    case UNDIST_CUDA:
      if(undist_frame_cuda_.empty())
      {
        std::cerr << "ERROR: Undistorted CUDA frame has not been set" << std::endl;
      }
      return undist_frame_cuda_;
    case HSV_CUDA:
      if(hsv_frame_cuda_.empty())
      {
        std::cerr << "ERROR: HSV CUDA frame has not been set" << std::endl;
      }
      return hsv_frame_cuda_;
  }
}

void System::SetFrames()
{
  if(frames_required_[HD])
  {
    if(!frame_exists_[HD])
    {
      std::cerr << "ERROR: HD Frame must be set manually using the SetHDFrame method!" << std::endl;
    }
  }
  if(frames_required_[SD])
  {
#if OPENCV_CUDA
    if(cuda_frame_exists_[SD_CUDA]) 
    {
      sd_frame_cuda_.download(sd_frame_);
    }
    else 
    {
      if (resize_scale_ != 1) 
      {
        cv::resize(hd_frame_, sd_frame_, sd_res_, 0, 0, cv::INTER_AREA);
      }
      else 
      {
        sd_frame_ = hd_frame_.clone();
      }
    }
#else
    if (resize_scale_ != 1) 
    {
      cv::resize(hd_frame_, sd_frame_, sd_res_, 0, 0, cv::INTER_AREA);
    }
    else 
    {
      sd_frame_ = hd_frame_.clone();
    }
#endif
    frame_exists_[SD] = true;
  }
  if(frames_required_[MONO])
  {
    cv::cvtColor(sd_frame_, mono_frame_, CV_RGB2GRAY);
    frame_exists_[MONO] = true;
  }
  if(frames_required_[UNDIST])
  {
    cv::undistort(sd_frame_, undist_frame_, sd_camera_matrix_, dist_coeff_);
    frame_exists_[UNDIST] = true;
  }
  if(frames_required_[HSV])
  {
    cv::cvtColor(sd_frame_, hsv_frame_, CV_BGR2HSV);
    frame_exists_[HSV] = true;
  }
}

void System::SetCUDAFrames()
{
  if(cuda_frames_required_[HD_CUDA])
  {
    hd_frame_cuda_.upload(hd_frame_);
    cuda_frame_exists_[HD_CUDA] = true;
  }
  if(cuda_frames_required_[SD_CUDA])
  {
    if (resize_scale_ != 1)
    {
      cv::cuda::resize(hd_frame_cuda_, sd_frame_cuda_, sd_res_, 0, 0, cv::INTER_AREA);
    }
    else 
    {
      hd_frame_cuda_.copyTo(sd_frame_cuda_);
    }
    cuda_frame_exists_[SD_CUDA] = true;
  }
  if(cuda_frames_required_[MONO_CUDA])
  {
    cv::cuda::cvtColor(sd_frame_cuda_, mono_frame_cuda_, CV_RGB2GRAY);
    cuda_frame_exists_[MONO_CUDA] = true;
  }
  if(cuda_frames_required_[UNDIST_CUDA])
  {
    cv::cuda::remap(sd_frame_cuda_, undist_frame_cuda_, undistort_map_x_, undistort_map_y_, cv::INTER_LINEAR);
    cuda_frame_exists_[UNDIST_CUDA] = true;
  }
  if(cuda_frames_required_[HSV_CUDA])
  {
    cv::cuda::cvtColor(sd_frame_cuda_, hsv_frame_cuda_, CV_BGR2HSV);
    cuda_frame_exists_[HSV_CUDA] = true;
  }
}

void System::RegisterPluginFrames(FrameRefVector& plugin_frames)
{
  for(int i = 0; i < num_frame_types_; i++)
  {
    if(frames_required_[i] != true && plugin_frames[i] == true)
    {
      frames_required_[i] = plugin_frames[i];
      if(frames_required_[i] == true && i == 0)
      {
        std::cout << "IMAGE ENABLED: HD" << std::endl;
      }
      else if(frames_required_[i] == true && i == 1)
      {
        std::cout << "IMAGE ENABLED: SD" << std::endl;
      }
      else if(frames_required_[i] == true && i == 2)
      {
        std::cout << "IMAGE ENABLED: MONO" << std::endl;
      }
      else if(frames_required_[i] == true && i == 3)
      {
        std::cout << "IMAGE ENABLED: UNDIST" << std::endl;
      }
      else if(frames_required_[i] == true && i == 4)
      {
        std::cout << "IMAGE ENABLED: HSV" << std::endl;
      }
    }
  }
}

void System::RegisterPluginCUDAFrames(FrameRefVector& plugin_frames_cuda)
{
  for(int i = 0; i < num_cuda_frame_types_; i++)
  {
    if(cuda_frames_required_[i] != true && plugin_frames_cuda[i] == true)
    {
      cuda_frames_required_[i] = plugin_frames_cuda[i];
      if(cuda_frames_required_[i] == true && i == 0)
      {
        std::cout << "CUDA IMAGE ENABLED: HD CUDA" << std::endl;
      }
      else if(cuda_frames_required_[i] == true && i == 1)
      {
        std::cout << "CUDA IMAGE ENABLED: SD CUDA" << std::endl;
      }
      else if(cuda_frames_required_[i] == true && i == 2)
      {
        std::cout << "CUDA IMAGE ENABLED: MONO CUDA" << std::endl;
      }
      else if(cuda_frames_required_[i] == true && i == 3)
      {
        std::cout << "CUDA IMAGE ENABLED: UNDIST CUDA" << std::endl;
      }
      else if(cuda_frames_required_[i] == true && i == 4)
      {
        std::cout << "CUDA IMAGE ENABLED: HSV CUDA" << std::endl;
      }
    }
  }
}

void System::ResetFrames()
{
  for(int i = 0; i < num_frame_types_; i++) 
  {
    frame_exists_[i] = false;
  }

  for(int i = 0; i < num_cuda_frame_types_; i++) 
  {
    cuda_frame_exists_[i] = false;
  }

}
  
}  //end namespace common