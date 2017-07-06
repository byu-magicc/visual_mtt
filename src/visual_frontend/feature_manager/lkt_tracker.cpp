#include "visual_frontend/feature_manager/lkt_tracker.h"

namespace visual_frontend {

int opencv_error_handler(int status, const char* func_name,
                         const char* err_msg, const char* file_name, int line,
                         void* userdata)
{
  // std::string err_msg_suppressed = "prevPyr[level * lvlStep1].size() == nextPyr[level * lvlStep2].size()";
  //
  // // display the OpenCV error message unless
  // std::cout << "OpenCV Error: " << "(" << err_msg << ")";
  // std::cout << " in " << func_name;
  // std::cout << ", file " << file_name;
  // std::cout << ", line " << line << std::endl;
  //
  // if (std::string(err_msg) == "prevPyr[level * lvlStep1].size() == nextPyr[level * lvlStep2].size()")
  // {
  //   std::cout << "true -----------------------" << std::endl;
  // }


  return 0; // return value is not used
}

// ----------------------------------------------------------------------------

LKTTracker::LKTTracker(double corner_quality, double corner_quality_min,
                       double corner_quality_max, double corner_quality_alpha,
                       int pyramid_size)

  : corner_quality_(corner_quality), corner_quality_min_(corner_quality_min),
    corner_quality_max_(corner_quality_max), corner_quality_alpha_(corner_quality_alpha)
{
  // Create a Good Features to Track feature detector
  gftt_detector_ = init_gftt();

  // Termination criteria for the OpenCV LK Optical Flow algorithm
  kltTerm_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.03);

  // The size of the search window at each pyramid level for LK Optical Flow
  pyramid_size_ = cv::Size(pyramid_size, pyramid_size);
}

// ----------------------------------------------------------------------------

void LKTTracker::find_correspondences(const cv::Mat& img, std::vector<cv::Point2f>& prev_matched, std::vector<cv::Point2f>& next_matched)
{

  // Convert to grayscale
  cv::Mat mono;
  cv::cvtColor(img, mono, CV_RGB2GRAY);

  //
  // Optical Flow for Feature Correspondences
  //

  // Uses previous GFTT features to find next_features in current frame
  std::vector<cv::Point2f> next_features;
  std::vector<unsigned char> valid;
  calculate_flow(mono, next_features, valid);

  // Only keep features that were matched in both frames
  for(int ii = 0; ii < valid.size(); ii++)
    if (valid[ii])
    {
      prev_matched.push_back(prev_features_[ii]);
      next_matched.push_back(next_features[ii]);
    }

  //
  // Find a new set of GFTT corners
  //

  // find fresh feature points
  std::vector<cv::Point2f> features;
  detect_features(mono, features);

  // save features for the next iteration.
  prev_features_.swap(features);
}

// ----------------------------------------------------------------------------

void LKTTracker::set_max_features(int points_max)
{
#ifndef OPENCV_CUDA
  gftt_detector_->setMaxFeatures(points_max);
#endif
}

// ----------------------------------------------------------------------------
// Private Methods
// ---------------------------------------------------------------------------

cv::Ptr<cvFeatureDetector_t> LKTTracker::init_gftt()
{
  const double minDistance = 10;
  const int blockSize = 3;
  const bool useHarrisDetector = false;

#ifdef OPENCV_CUDA
  const int points_max = 10000;
  return cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, points_max, corner_quality_, minDistance, blockSize, useHarrisDetector);
#else
  return cv::GFTTDetector::create(0, corner_quality_, minDistance, blockSize, useHarrisDetector);
#endif
}

// ---------------------------------------------------------------------------

void LKTTracker::calculate_flow(const cv::Mat& mono, std::vector<cv::Point2f>& next_features, std::vector<unsigned char>& valid)
{

#if OPENCV_CUDA
  static cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> gSparsePyrLK = cv::cuda::SparsePyrLKOpticalFlow::create(pyramid_size_, 3, 20);
#endif

#ifndef OPENCV_CUDA
  // Build optical flow pyramids for current image
  std::vector<cv::Mat> current_pyramids;
  buildOpticalFlowPyramid(mono, current_pyramids, pyramid_size_, 2);
#endif

  // suppress the pyramid size conflict error message
  cv::redirectError(opencv_error_handler);

  if (!first_image_)
  {
    try
    {
#ifdef OPENCV_CUDA
      // Upload images to GPU
      cv::cuda::GpuMat gLastMono(last_mono_);
      cv::cuda::GpuMat gMono(mono);

      // Upload previous features to GPU
      cv::cuda::GpuMat gPrevFeatures;
      gpu::upload(prev_features_, gPrevFeatures);

      // Run LK optical flow on the GPU
      cv::cuda::GpuMat gNextFeatures, gValid;
      gSparsePyrLK->calc(gLastMono, gMono, gPrevFeatures, gNextFeatures, gValid);

      // Download from the GPU
      gpu::download(gNextFeatures, next_features);
      gpu::download(gValid, valid);

#else
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(last_pyramids_, current_pyramids,
                               prev_features_, next_features,
                               valid, err, pyramid_size_, 3, kltTerm_, 0, 1e-4);
#endif
    }
    catch (...)
    {
      valid.clear();
      ROS_WARN("lkt tracker: pyramid mismatch, skipping this iteration");
    }
  }
  else
  {
    first_image_ = false;
  }

  // reset opencv error messages to standard
  cv::redirectError(nullptr);

#ifdef OPENCV_CUDA
  // save mono for the next iteration
  last_mono_ = mono.clone();
#else
  // save pyramids for the next iteration.
  last_pyramids_ = current_pyramids;
#endif
}

// ---------------------------------------------------------------------------

void LKTTracker::detect_features(const cv::Mat& mono, std::vector<cv::Point2f>& features)
{
  #ifdef OPENCV_CUDA

    cv::cuda::GpuMat gMono(mono);
    cv::cuda::GpuMat gFeatures;
    gftt_detector_->detect(gMono, gFeatures);

    // Download
    gpu::download(gFeatures, features);

  #else

    std::vector<cv::KeyPoint> keypoints;
    gftt_detector_->detect(mono, keypoints);

    // Unpack keypoints and create regular features points
    features.resize(keypoints.size());
    for (auto&& key : keypoints)
      features.push_back(key.pt);

  #endif
}

}