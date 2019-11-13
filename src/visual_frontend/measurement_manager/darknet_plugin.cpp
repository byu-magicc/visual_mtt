#include "visual_frontend/measurement_manager/darknet_plugin.h"

namespace visual_frontend {


#ifdef VMTT_FILE_PATH
const std::string kVmttFilePath_ = VMTT_FILE_PATH;
#else
#error Path of visual mtt repository is not defined in CMakeLists.txt.
#endif


DarknetPlugin::DarknetPlugin()
{

    name_  = "Darknet";
    id_ = 2;            
    has_velocity_= false; 
    sigmaR_pos_ = 0.007;
    sigmaR_vel_ = 0; 
    sequence_ = 0;
    drawn_ = false;

// Required frames for plugin
#if OPENCV_CUDA
  frames_required_ = {true, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, false, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#else
  frames_required_ = {true, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
#endif

}

// ----------------------------------------------------------------------------


DarknetPlugin::~DarknetPlugin()
{
    DestroyWindows();

}

// ----------------------------------------------------------------------------

void DarknetPlugin::Initialize(const common::Params& params)
{


	std::string labels_file_path, config_file_path, weights_file_path, params_file_path;
	std::string labels_file_name, config_file_name, weights_file_name, params_file_name;
	
	// Get the file names
  params.GetParam("measurement_manger/darknet_plugin/labels_filename", labels_file_name, "");
  params.GetParam("measurement_manger/darknet_plugin/config_filename", config_file_name, "");
  params.GetParam("measurement_manger/darknet_plugin/weights_filename", weights_file_name,"");
  params.GetParam("measurement_manger/darknet_plugin/params_filename", params_file_name, "");

  // Construct the file paths
  labels_file_path =  kVmttFilePath_ + labels_file_name;
  config_file_path =  kVmttFilePath_ + config_file_name;
  weights_file_path = kVmttFilePath_ + weights_file_name;
  params_file_path =  kVmttFilePath_ + params_file_name;

  // Initialize YOLO
  yolo.Initialize(
    labels_file_path,
    params_file_path,
    config_file_path,
    weights_file_path);

  // Register the subscriber
  yolo.Subscriber(&DarknetPlugin::YoloCallback,this);

}

// ----------------------------------------------------------------------------

void DarknetPlugin::SetParameters(const visual_mtt::visual_frontendConfig& config)
{
  darknet_wrapper::common::DynamicParams params;

  params.threshold = config.darknet_threshold;
  params.frame_stride = config.darknet_frame_stride;
  params.draw_detections = config.darknet_draw_detections;
  
  ShouldReset(config.darknet_enabled);

  yolo.SetDynamicParams(params);

}

// ----------------------------------------------------------------------------

void DarknetPlugin::DrawMeasurements(const common::System& sys)
{

  if (!drawn_img_.empty())
  {

    if(drawn_ == false)
    {
      cv::namedWindow(name_,CV_WINDOW_NORMAL);
    }
    drawn_= true;
    cv::imshow(name_,drawn_img_);
    cv::resizeWindow(name_,sys.sd_res_.width,sys.sd_res_.height);

  }

}

// ----------------------------------------------------------------------------

bool DarknetPlugin::GenerateMeasurements(const common::System& sys)
{

  // Send the next image
  std::cout << "frame empty: " << sys.GetFrame(common::HD).empty() << " : " << sys.GetFrame(common::HD).size() << std::endl;
  yolo.ImageCallback(sys.GetFrame(common::HD),sequence_);
  sequence_++;

  meas_pos_.clear();

  // Undistort points and project them onto the normalized image plane
  if(d_curr_points_.size() > 0)
  {
    std::lock_guard<std::mutex> lock(yolo_callback_mutex_);
    cv::undistortPoints(d_curr_points_, meas_pos_, sys.hd_camera_matrix_, sys.dist_coeff_);
  }

  return static_cast<bool>(meas_pos_.size());

}

// ----------------------------------------------------------------------------

void DarknetPlugin::Reset()
{
  DestroyWindows();
  drawn_ = false;
}

// ----------------------------------------------------------------------------

void DarknetPlugin::YoloCallback(const cv::Mat& img, const darknet_wrapper::common::BoundingBoxes& boxes, const int& seq)
{
  drawn_img_ = img;
  boxes_ = boxes;
  img_seq_ = seq;

  cv::Point2f point;

  d_curr_points_.clear();

  // Extract the points using the lockgaurd to protect another thread from 
  // accessing it. 
  std::lock_guard<std::mutex> lock(yolo_callback_mutex_);
  for (auto& box : boxes.boxes)
  {
    point = cv::Point2f(box.xcent, box.ycent);
    d_curr_points_.push_back(point);
  }

}

// ----------------------------------------------------------------------------

void DarknetPlugin::DestroyWindows()
{
  if (drawn_)
    cv::destroyWindow(name_);
}

}



// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::DarknetPlugin, visual_frontend::MeasurementBase)