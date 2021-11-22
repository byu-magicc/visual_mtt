#include "common/system.h"

namespace common {



// Initialize static member of class System
std::string System::picture_file_path_ = "";
bool System::picture_file_path_set_ = false;


System::System()
{
  // Init flags
  ClearFlags();

  resize_scale_ = 1;
  num_of_measurements_ = 0;
  cam_info_received_ = false;
  tuning_ = false;
  picture_file_path_set_ = false;

  // Required default frames
  default_frames_required_ = {true, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
#if OPENCV_CUDA
  default_cuda_frames_required_ = {true, false, false, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#endif

  // Initialize all required frames as false
  for(int i = 0; i < num_frame_types_; i++)
  {
    frames_required_[i] = false;
  }
#if OPENCV_CUDA
  for(int i = 0; i < num_cuda_frame_types_; i++)
  {
    cuda_frames_required_[i] = false;
  }
#endif

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

  // boundary_d is a polygon in the original sd frame, put in matrix form
  cv::Mat boundary_mat(boundary_d);

  // build the mask
  boundary_mat.convertTo(boundary_mat, CV_32SC1);
  undistorted_region_mask_ = cv::Mat(sd_res_, CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(undistorted_region_mask_, boundary_mat, cv::Scalar(255));


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

void System::AddMeasurements(const rransac::SourceParameters& source_params, const bool has_velocity, 
                             const std::vector<cv::Point2f>& meas_pos, 
                             const std::vector<cv::Point2f>& meas_vel)
{

RR_Measurement m;
m.transform_state = false;
m.transform_meas = false;
m.source_index = source_params.source_index_;
m.type = source_params.type_;
m.time_stamp = this->current_time_;
// std::cout << "curr time: " << m.time_stamp << std::endl;
Eigen::Matrix<double,2,1> tmp;

for (unsigned int ii=0; ii < meas_pos.size(); ++ii ) {

  tmp(0) = meas_pos[ii].x;
  tmp(1) = meas_pos[ii].y;
  m.pose = tmp;

  if (has_velocity) {
    tmp << meas_vel[ii].x, meas_vel[ii].y;
    m.twist = tmp;
  }

  measurements_.push_back(m);
  num_of_measurements_++;

}



}

// --------------------------------------------------------------------------------------

void System::SetMeasurementFlag(const bool& good_measurements)
{
  good_measurements_ = good_measurements;
}

// --------------------------------------------------------------------------------------

void System::SetPictureFilepath(const std::string& picture_file_path)
{
  if(picture_file_path == "")
  {
    ROS_WARN_STREAM("System:: Filepath has not been set. Pictures cannot be saved.");
    picture_file_path_set_ = false;
  }
  else
  {
    picture_file_path_ = picture_file_path;
    picture_file_path_set_ = true;
  }
}

// --------------------------------------------------------------------------------------

void System::TakePicture(int event, int x, int y, int, void* param)
{
  // Only save a picture if the event is a double click and
  // the filepath is set.
  if( event != cv::EVENT_LBUTTONDBLCLK  || !picture_file_path_set_)
    return;
  
  PictureParams* pp = (PictureParams*)param;

  char full_file_name[100];
  sprintf(full_file_name,"%s%s%05x.png",picture_file_path_.c_str(), pp->file_name.c_str(),pp->pic_num);

  // Make sure that the image isn't empty
  if(!pp->img.empty())
  {
    cv::imwrite( full_file_name, pp->img );
    pp->pic_num++;
  }

}

// --------------------------------------------------------------------------------------

cv::Mat System::GetFrame(frame_type_ frame_type) const
{
  // return requested frame type or error message if it hasn't been set at iteration
  switch(frame_type)
  {
    case HD:
      if(!frame_exists_[HD])
      {
        std::cerr << "ERROR: HD frame has not been set" << std::endl;
      }
      return hd_frame_;
    case SD:
      if(!frame_exists_[SD])
      {
        std::cerr << "ERROR: SD frame has not been set" << std::endl;
      }
      return sd_frame_;
    case MONO:
      if(!frame_exists_[MONO])
      {
        std::cerr << "ERROR: Mono frame has not been set" << std::endl;
      }
      return mono_frame_;
    case UNDIST:
      if(!frame_exists_[UNDIST])
      {
        std::cerr << "ERROR: Undistorted frame has not been set" << std::endl;
      }
      return undist_frame_;
    case HSV:
      if(!frame_exists_[HSV])
      {
        std::cerr << "ERROR: HSV frame has not been set" << std::endl;
      }
      return hsv_frame_;
  }
  std::cerr << "ERROR: HD frame has not been set" << std::endl;
  return hd_frame_;
}

#if OPENCV_CUDA
cv::cuda::GpuMat System::GetCUDAFrame(frame_type_cuda_ frame_type_cuda) const
{
  // return requested CUDA frame type or error message if it hasn't been set at iteration
  switch(frame_type_cuda)
  {
    case HD_CUDA:
      if(!cuda_frame_exists_[HD_CUDA])
      {
        std::cerr << "ERROR: HD CUDA frame has not been set" << std::endl;
      }
      return hd_frame_cuda_;
    case SD_CUDA:
      if(!cuda_frame_exists_[SD_CUDA])
      {
        std::cerr << "ERROR: SD CUDA frame has not been set" << std::endl;
      }
      return sd_frame_cuda_;
    case MONO_CUDA:
      if(!cuda_frame_exists_[MONO_CUDA])
      {
        std::cerr << "ERROR: Mono CUDA frame has not been set" << std::endl;
      }
      return mono_frame_cuda_;
    case UNDIST_CUDA:
      if(!cuda_frame_exists_[UNDIST_CUDA])
      {
        std::cerr << "ERROR: Undistorted CUDA frame has not been set" << std::endl;
      }
      return undist_frame_cuda_;
    case HSV_CUDA:
      if(!cuda_frame_exists_[HSV_CUDA])
      {
        std::cerr << "ERROR: HSV CUDA frame has not been set" << std::endl;
      }
      return hsv_frame_cuda_;
  }
  std::cerr << "ERROR: HD CUDA frame has not been set" << std::endl;
  return hd_frame_cuda_;
}
#endif

void System::SetFrames()
{
  // HD frame must exist or sends error message
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
    // download sd_frame from GPU if available, otherwise resize hd_frame
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
    // copy hd_frame if no resize, otherwise resize hd_frame
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

#if OPENCV_CUDA
void System::SetCUDAFrames()
{
  if(cuda_frames_required_[HD_CUDA])
  {
    hd_frame_cuda_.upload(hd_frame_);
    cuda_frame_exists_[HD_CUDA] = true;
  }
  if(cuda_frames_required_[SD_CUDA])
  {
    // copy hd_frame_cuda_ on GPU if no resize, otherwise resize hd_frame_cuda_ on GPU
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
#endif

void System::RegisterPluginFrames(FrameRefVector& plugin_frames)
{
  for(int i = 0; i < num_frame_types_; i++)
  {
    // print frame types requested
    if(plugin_frames[i] && !(frames_required_[i]) && i == 0)
    {
      std::cout << "IMAGE REQUESTED: HD" << std::endl;
    }
    else if(plugin_frames[i] && !(frames_required_[i]) && i == 1)
    {
      std::cout << "IMAGE REQUESTED: SD" << std::endl;
    }
    else if(plugin_frames[i] && !(frames_required_[i]) && i == 2)
    {
      std::cout << "IMAGE REQUESTED: MONO" << std::endl;
    }
    else if(plugin_frames[i] && !(frames_required_[i]) && i == 3)
    {
      std::cout << "IMAGE REQUESTED: UNDIST" << std::endl;
    }
    else if(plugin_frames[i] && !(frames_required_[i]) && i == 4)
    {
      std::cout << "IMAGE REQUESTED: HSV" << std::endl;
    }
    
    // integrate requested frames into previously requested frames
    frames_required_[i] = frames_required_[i] | plugin_frames[i];

    // add SD frame if any derived frames needed
    if(i == 2 || i == 3 || i == 4)
    {
      if(!(frames_required_[1]))
      {
        std::cout << "IMAGE REQUESTED: SD" << std::endl;
        frames_required_[1] = true;
      }
    }
  }
}

#if OPENCV_CUDA
void System::RegisterPluginCUDAFrames(CUDAFrameRefVector& plugin_frames_cuda)
{
  // print CUDA frame types requested
  for(int i = 0; i < num_cuda_frame_types_; i++)
  {
    if(plugin_frames_cuda[i] && !(cuda_frames_required_[i]) && i == 0)
    {
      std::cout << "CUDA IMAGE REQUESTED: HD CUDA" << std::endl;
    }
    else if(plugin_frames_cuda[i] && !(cuda_frames_required_[i]) && i == 1)
    {
      std::cout << "CUDA IMAGE REQUESTED: SD CUDA" << std::endl;
    }
    else if(plugin_frames_cuda[i] && !(cuda_frames_required_[i]) && i == 2)
    {
      std::cout << "CUDA IMAGE REQUESTED: MONO CUDA" << std::endl;
    }
    else if(plugin_frames_cuda[i] && !(cuda_frames_required_[i]) && i == 3)
    {
      std::cout << "CUDA IMAGE REQUESTED: UNDIST CUDA" << std::endl;
    }
    else if(plugin_frames_cuda[i] && !(cuda_frames_required_[i]) && i == 4)
    {
      std::cout << "CUDA IMAGE REQUESTED: HSV CUDA" << std::endl;
    }
    
    // integrate requested CUDA frames into previously requested CUDA frames
    cuda_frames_required_[i] = cuda_frames_required_[i] | plugin_frames_cuda[i];

    // add CUDA SD frame if any derived frames needed
    if(i == 2 || i == 3 || i == 4)
    {
      if(!(cuda_frames_required_[1]))
      {
        std::cout << "CUDA IMAGE REQUESTED: SD CUDA" << std::endl;
        cuda_frames_required_[1] = true;
      }
    }
  }
}
#endif

void System::RegisterFrame(frame_type_ frame_type)
{
  // register requested frame type
  frames_required_[frame_type] = true;

  // add SD frame if derivative frame needed
  if(frame_type == frame_type_::MONO || frame_type == frame_type_::UNDIST || frame_type == frame_type_::HSV)
  {
    if(!(frames_required_[frame_type_::SD]))
    {
      frames_required_[frame_type_::SD] = true;
    }
  }

  switch(frame_type)
  {
    case HD:
      std::cout << "IMAGE REQUESTED: HD" << std::endl;
      break;
    case SD:
      std::cout << "IMAGE REQUESTED: SD" << std::endl;
      break;
    case MONO:
      std::cout << "IMAGE REQUESTED: MONO" << std::endl;
      break;
    case UNDIST:
      std::cout << "IMAGE REQUESTED: UNDIST" << std::endl;
      break;
    case HSV:
      std::cout << "IMAGE REQUESTED: HSV" << std::endl;
      break;
  }
}

#if OPENCV_CUDA
void System::RegisterCUDAFrame(frame_type_cuda_ frame_type_cuda)
{
  // register requested CUDA frame type
  cuda_frames_required_[frame_type_cuda] = true;

  // add SD CUDA frame if derivative frame needed
  if(frame_type_cuda == frame_type_cuda_::MONO_CUDA || frame_type_cuda == frame_type_cuda_::UNDIST_CUDA || frame_type_cuda == frame_type_cuda_::HSV_CUDA)
  {
    if(!(cuda_frames_required_[frame_type_cuda_::SD_CUDA]))
    {
      cuda_frames_required_[frame_type_cuda_::SD_CUDA] = true;
    }
  }

  switch(frame_type_cuda)
  {
    case HD_CUDA:
      std::cout << "CUDA IMAGE REQUESTED: HD_CUDA" << std::endl;
      break;
    case SD_CUDA:
      std::cout << "CUDA IMAGE REQUESTED: SD_CUDA" << std::endl;
      break;
    case MONO_CUDA:
      std::cout << "CUDA IMAGE REQUESTED: MONO_CUDA" << std::endl;
      break;
    case UNDIST_CUDA:
      std::cout << "CUDA IMAGE REQUESTED: UNDIST_CUDA" << std::endl;
      break;
    case HSV_CUDA:
      std::cout << "CUDA IMAGE REQUESTED: HSV_CUDA" << std::endl;
      break;
  }
}
#endif

void System::ResetFrames()
{
  for(int i = 0; i < num_frame_types_; i++) 
  {
    frame_exists_[i] = false;
  }

#if OPENCV_CUDA
  for(int i = 0; i < num_cuda_frame_types_; i++) 
  {
    cuda_frame_exists_[i] = false;
  }
#endif
}
  
}  //end namespace common