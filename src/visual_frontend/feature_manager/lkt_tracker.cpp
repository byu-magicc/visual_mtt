#include "visual_frontend/feature_manager/lkt_tracker.h"

namespace visual_frontend {

LKTTracker::LKTTracker()
{
  name_ = "LKTTracker";
  enabled_ = "false";
  drawn_ = false;
  first_image_ = true;

  pic_params_.pic_num = 0;
  pic_params_.file_name = name_;
  max_features_ = 0;

// Required frames for plugin
#if OPENCV_CUDA
  frames_required_ = {false, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, true, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#else
  frames_required_ = {false, false, true, false, false};  // {HD, SD, MONO, UNDIST, HSV}
#endif
}

// ----------------------------------------------------------------------------

LKTTracker::~LKTTracker()
{
  DestroyWindows();
}

// ----------------------------------------------------------------------------

void LKTTracker::Initialize(const common::Params& params) 
{

  double pyramid_size;
  params.GetParam("feature_manager/lkt_tracker/corner_quality", corner_quality_, 0.03);
  params.GetParam("feature_manager/lkt_tracker/corner_quality_alpha", corner_quality_alpha_, 0.03);
  params.GetParam("feature_manager/lkt_tracker/pyramid_size", pyramid_size, 21.0);


  // Create a Good Features to Track feature detector
  gftt_detector_ = InitGftt();

  // Termination criteria for the OpenCV LK Optical Flow algorithm
  kltTerm_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.03);

  // The size of the search window at each pyramid level for LK Optical Flow
  pyramid_size_ = cv::Size(pyramid_size, pyramid_size);

}

// ----------------------------------------------------------------------------

void LKTTracker::SetParameters(const visual_mtt::visual_frontendConfig& config) 
{


  // Update max features parameter if it has changed.
  if (max_features_ != config.lkt_tracker_max_features) {
    max_features_ = config.lkt_tracker_max_features;
    SetMaxFeatures(max_features_);
  }

  // see if the Feature Extractor needs to be reset. 
  ShouldReset(config.lkt_tracker_enabled);


}

// ----------------------------------------------------------------------------

void LKTTracker::DrawFeatures(const common::System& sys)
{

  // Since LKT_tracker can generate a lot of features, we want to limit the
  // amount actually drawn by *draw_num*. Also, we want to evenly sample the
  // features that we draw. This is the purpose of *inc*.
  int draw_num = 50;
  int inc = (draw_num < d_prev_matched_.size()) ? round(d_prev_matched_.size()/draw_num) : 1;

  cv::Mat draw = (sys.GetFrame(common::SD)).clone();

  float scale = 1.2;

  // plot measurements
  for (int j=0; j<d_prev_matched_.size(); j+=inc)
  {
    cv::Point2f scaled_point = d_prev_matched_[j] + (d_curr_matched_[j]-d_prev_matched_[j])*scale;
    cv::arrowedLine(draw, d_prev_matched_[j], d_curr_matched_[j], cv::Scalar(255, 0, 255), 2, CV_AA);
  }

  if(drawn_ == false)
  {
    cv::namedWindow(name_);
    cv::setMouseCallback(name_,sys.TakePicture, &pic_params_);
  }

  if (!draw.empty())
  {
    drawn_ = true;
    cv::imshow(name_, draw);
    pic_params_.img = draw.clone();
  }

}

// ----------------------------------------------------------------------------

bool LKTTracker::FindCorrespondences(const common::System& sys)
{

  bool good_features = false;

  // Clear history
  d_prev_matched_.clear();
  d_curr_matched_.clear();

  //
  // Optical Flow for Feature Correspondences
  //

  // Uses previous GFTT features to find next_features in current frame
  std::vector<cv::Point2f> d_curr_features;
  std::vector<unsigned char> valid;
  valid.clear();

  #if OPENCV_CUDA
    CalculateFlow(sys.GetCUDAFrame(common::MONO_CUDA), d_curr_features, valid, sys);
  #else
    CalculateFlow(sys.GetFrame(common::MONO), d_curr_features, valid, sys);
  #endif

  // Only keep features that were matched in both frames
  for(int ii = 0; ii < valid.size(); ii++)
    if (valid[ii])
    {
      d_prev_matched_.push_back(d_prev_features_[ii]);
      d_curr_matched_.push_back(d_curr_features[ii]);
    }

  //
  // Find a new set of GFTT corners
  //
  
  d_prev_features_.clear();
  #if OPENCV_CUDA
    DetectFeatures(sys.GetCUDAFrame(common::MONO_CUDA), d_prev_features_, sys.undistorted_region_mask_);
  #else
    DetectFeatures(sys.GetFrame(common::MONO), d_prev_features_, sys.undistorted_region_mask_);
  #endif

  if (d_prev_matched_.size() > 10)
    good_features = true;
  else
    ROS_WARN_STREAM_THROTTLE(sys.message_output_period_,"LKTTracker: Not enough features matched. Bad features.");

  return good_features;
}

// ----------------------------------------------------------------------------
// Private Methods
// ---------------------------------------------------------------------------

void LKTTracker::Reset()
{
  DestroyWindows();
  drawn_ = false;
  first_image_ = true;

}

// ---------------------------------------------------------------------------

cv::Ptr<cvFeatureDetector_t> LKTTracker::InitGftt(int max_features)
{
  const double minDistance = 10;
  const int blockSize = 3;
  const bool useHarrisDetector = false;

#ifdef OPENCV_CUDA
  return cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, max_features, corner_quality_, minDistance, blockSize, useHarrisDetector);
#else
  return cv::GFTTDetector::create(0, corner_quality_, minDistance, blockSize, useHarrisDetector);
#endif
}

// ---------------------------------------------------------------------------

void LKTTracker::CalculateFlow(const cv::Mat& mono, std::vector<cv::Point2f>& curr_features, std::vector<unsigned char>& valid,const common::System& sys)
{
  // Build optical flow pyramids for current image
  std::vector<cv::Mat> current_pyramids;
  cv::buildOpticalFlowPyramid(mono, current_pyramids, pyramid_size_, 2);

  if (!first_image_)
  {
    try
    {
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(last_pyramids_, current_pyramids,
                               d_prev_features_, curr_features,
                               valid, err, pyramid_size_, 3, kltTerm_, 0, 1e-4);
    }
    catch (...)
    {
      valid.clear();
      ROS_WARN_STREAM_THROTTLE(sys.message_output_period_,"lkt tracker: pyramid mismatch, skipping this iteration");
    }
  }
  else
  {
    first_image_ = false;
  }
  // save pyramids for the next iteration.
  last_pyramids_ = current_pyramids;

}

#if OPENCV_CUDA
void LKTTracker::CalculateFlow(const cv::cuda::GpuMat& gMono, std::vector<cv::Point2f>& curr_features, std::vector<unsigned char>& valid,const common::System& sys)
{
  static cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> gSparsePyrLK = cv::cuda::SparsePyrLKOpticalFlow::create(pyramid_size_, 3, 30, false);

  if (!first_image_)
  {
    try
    {
      // Upload previous features to GPU
      cv::cuda::GpuMat gPrevFeatures;
      common::gpu::upload(d_prev_features_, gPrevFeatures);

      // Run LK optical flow on the GPU
      cv::cuda::GpuMat gNextFeatures, gValid;
      gSparsePyrLK->calc(gLastMono, gMono, gPrevFeatures, gNextFeatures, gValid);

      // Download from the GPU
      common::gpu::download(gNextFeatures, curr_features);
      common::gpu::download(gValid, valid);
    }
    catch (...)
    {
      valid.clear();
      ROS_WARN_STREAM_THROTTLE(sys.message_output_period_,"lkt tracker: pyramid mismatch, skipping this iteration");
    }
  }
  else
  {
    first_image_ = false;
  }



  // save mono for the next iteration
  gLastMono = gMono.clone();
}
#endif
// ---------------------------------------------------------------------------



#if OPENCV_CUDA
void LKTTracker::DetectFeatures(const cv::cuda::GpuMat& gMono, std::vector<cv::Point2f>& features, const cv::Mat& mask)
{
    cv::cuda::GpuMat gMask(mask);
    cv::cuda::GpuMat gFeatures;
    gftt_detector_->detect(gMono, gFeatures, gMask);
    


    // Download
    common::gpu::download(gFeatures, features);
    // for (auto& f : features) 
    //   std::cout << f << std::endl;
}

#else

void LKTTracker::DetectFeatures(const cv::Mat& mono, std::vector<cv::Point2f>& features, const cv::Mat& mask)
{   
  std::vector<cv::KeyPoint> keypoints;
  gftt_detector_->detect(mono, keypoints, mask);

  // Unpack keypoints and create regular features points
  features.resize(keypoints.size());
  for (auto&& key : keypoints)
    features.push_back(key.pt);
}


#endif

// ---------------------------------------------------------------------------

void LKTTracker::SetMaxFeatures(int max_features)
{
#ifdef OPENCV_CUDA
  gftt_detector_ = InitGftt(max_features);
#else
  gftt_detector_->setMaxFeatures(max_features);
#endif

}

// ---------------------------------------------------------------------------

void LKTTracker::DestroyWindows()
{
  if(drawn_)
    cv::destroyWindow(name_);
}

}

// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::LKTTracker, visual_frontend::FeatureBase)