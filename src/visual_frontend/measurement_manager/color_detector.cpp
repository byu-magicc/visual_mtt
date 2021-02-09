#include "visual_frontend/measurement_manager/color_detector.h"

namespace visual_frontend {

ColorDetector::ColorDetector()
{

  enabled_ = false;
  name_ = "Color Detector";
  cv_window_name_ = name_ +" Threshold";
  drawn_ = false;
  has_velocity_ = false;
  source_parameters_changed_ = false;


#if TRACKING_SE2
  source_parameters_.type_ = rransac::MeasurementTypes::SEN_POS;
#else
  source_parameters_.type_ = rransac::MeasurementTypes::RN_POS;  
#endif
  
  source_parameters_.meas_cov_ = Eigen::Matrix<double,2,2>::Identity()*0.1;
  source_parameters_.spacial_density_of_false_meas_ = 0.01;
  source_parameters_.probability_of_detection_ = 0.95;
  source_parameters_.gate_probability_ = 0.9;
  source_parameters_.RANSAC_inlier_probability_ = 0.9;

  // Init params for the simple blob detector
  // We only want it to detect blobs according to size
  params_.filterByArea=true;
  params_.filterByCircularity = false;
  params_.filterByConvexity = false;
  params_.filterByInertia = false;
  params_.maxArea = 1000;      
  params_.minArea = 200;       
  // params_.minRepeatability = 100;         // I don't know what this does yet

  // Simple blob detector creates 
  params_.minThreshold = 0;             // I don't know what this does yet
  params_.maxThreshold = 255;             // I don't know what this does yet
  params_.thresholdStep = 50;           // I don't know what this does yet

// Required frames for plugin
#if OPENCV_CUDA
  frames_required_ = {false, true, false, false, true};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, false, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#else
  frames_required_ = {false, true, false, false, true};  // {HD, SD, MONO, UNDIST, HSV}
#endif

}


// ----------------------------------------------------------------------------

ColorDetector::~ColorDetector()
{
  DestroyWindows();
}

// ----------------------------------------------------------------------------

void ColorDetector::Initialize(const common::Params& params, const unsigned int source_index) {
  source_parameters_.source_index_ = source_index;
}

// ----------------------------------------------------------------------------

void ColorDetector::SetParameters(const visual_mtt::visual_frontendConfig& config)
{
  min_hue_ = config.color_detector_min_hue;
  max_hue_ = config.color_detector_max_hue;
  min_sat_ = config.color_detector_min_sat;
  max_sat_ = config.color_detector_max_sat;
  min_val_ = config.color_detector_min_val;
  max_val_ = config.color_detector_max_val;
  morph_iterations_ = config.color_detector_morph_iterations;
  morph_kernel_ = config.color_detector_morph_kernel;

  // Create a new detector
  if (params_.minArea!= config.color_detector_min_blob_size || params_.maxArea !=config.color_detector_max_blob_size) {

    params_.minArea = config.color_detector_min_blob_size;
    params_.maxArea = config.color_detector_max_blob_size;
    detector_ = cv::SimpleBlobDetector::create(params_);
  }

  ShouldReset(config.color_detector_enabled);

}

// ----------------------------------------------------------------------------

void ColorDetector::DrawMeasurements(const common::System& sys)
{
  cv::Mat draw = sys.GetFrame(common::SD).clone();

  drawKeypoints(draw,keypoints_,draw, cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  if(drawn_ == false)
  {
    cv::namedWindow(name_);
    cv::namedWindow(cv_window_name_);
    // cv::namedWindow("temp");
    cv::setMouseCallback(name_,sys.TakePicture, &pic_params_);
  }

  if (!draw.empty())
  {

    drawn_ = true;
    cv::imshow(name_, draw);
    cv::imshow(cv_window_name_, thresholded_img_);
    
    pic_params_.img = draw.clone();
  }
}

// ----------------------------------------------------------------------------

bool ColorDetector::GenerateMeasurements(const common::System& sys)
{

  // The maximum valuse of minArea must be less or equal to the
  // minimum image resolution
  if (params_.minArea > std::min(sys.sd_res_.height,sys.sd_res_.width))
  {
    ROS_WARN_STREAM("ColorDetector: min_blob_size > min(img.height,img.width). Setting min_blob_size = min(img.height,img.width)");
    params_.minArea = std::min(sys.sd_res_.height,sys.sd_res_.width);
    detector_ = cv::SimpleBlobDetector::create(params_);
  }

  bool good_measurements = false;
  meas_pos_.clear();
  d_meas_pos_.clear();

  // If the pixel falue is within this range, set it to 255 (white) else set it to 0 (black)
  cv:: inRange(sys.GetFrame(common::HSV), cv::Scalar(min_hue_, min_sat_, min_val_), cv::Scalar(max_hue_, max_sat_, max_val_), thresholded_img_);


  //morphological opening (remove small objects from the foreground)
  cv::erode(thresholded_img_, thresholded_img_, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_, morph_kernel_)),cv::Point(-1,-1),morph_iterations_ );
  cv::dilate(thresholded_img_, thresholded_img_, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_, morph_kernel_)),cv::Point(-1,-1),morph_iterations_ ); 

   //morphological closing (fill small holes in the foreground)
  cv::dilate( thresholded_img_, thresholded_img_, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_, morph_kernel_)),cv::Point(-1,-1),morph_iterations_ ); 
  cv::erode(thresholded_img_, thresholded_img_, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_kernel_, morph_kernel_)),cv::Point(-1,-1),morph_iterations_ );


  cv::threshold(thresholded_img_, inv_img_, 50,255,cv::THRESH_BINARY_INV);

  // The blob detector wont detect blobs that extend to edges. A rectangle is drawn
  // around the edge of the image to ensure that blobs that extend to the edge no 
  // no longer do and so they can be detected.
  cv::rectangle(inv_img_,cv::Point(0,0),sys.sd_res_, cv::Scalar(255), 4);

  detector_->detect(inv_img_,keypoints_);

  for (auto& kp : keypoints_ )
  {
    d_meas_pos_.push_back(kp.pt);
  }




  if (d_meas_pos_.size() > 0)
  {
      cv::undistortPoints(d_meas_pos_, meas_pos_, sys.sd_camera_matrix_, sys.dist_coeff_);
      good_measurements = true;
  }
  else
    ROS_DEBUG_STREAM_THROTTLE(sys.message_output_period_,"ColorDetector: No measurements found!");

  return good_measurements;  

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void ColorDetector::Reset() {

  DestroyWindows();
  drawn_ = false;
  first_image_ = true;

}

// ----------------------------------------------------------------------------


void ColorDetector::DestroyWindows() {

  if (drawn_)
    cv::destroyWindow(name_);
    cv::destroyWindow(cv_window_name_);
}

}

// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::ColorDetector, visual_frontend::MeasurementBase)