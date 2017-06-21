#include "visual_frontend/feature_manager/lkt_tracker.h"

namespace visual_frontend {

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

  // Build optical flow pyramids for current image
  std::vector<cv::Mat> current_pyramids;
  buildOpticalFlowPyramid(mono, current_pyramids, pyramid_size_, 2);

  if (!first_image_)
  {
    std::vector<cv::Point2f> next_features;
    std::vector<unsigned char> status;
    std::vector<float> err;

    // match feature points using lk algorithm
    cv::calcOpticalFlowPyrLK(last_pyramids_, current_pyramids,
                             prev_features_, next_features,
                             status, err, pyramid_size_, 3, kltTerm_, 0, 1e-4);

    // store only matched features
    for(int ii = 0; ii < status.size(); ii++)
      if (status[ii])
      {
        prev_matched.push_back(prev_features_[ii]);
        next_matched.push_back(next_features[ii]);
      }

  }
  else
  {
    first_image_ = false;
  }

  // RUN EVERY FRAME:

  // find fresh feature points
  std::vector<cv::KeyPoint> features;
  gftt_detector_->detect(mono, features);


#ifndef OPENCV_CUDA
  // perform adaptive corner quality using discrete alpha filtering
  // first determine the direction based on the number of features found
  int quality_step_dir = 0;
  if (features.size() < points_target_)
    quality_step_dir = -1;
  else
    quality_step_dir = 1;

  // apply alpha filter and upper/lower bounds
  corner_quality_ = corner_quality_*corner_quality_alpha_ + quality_step_dir*(1-corner_quality_alpha_);
  corner_quality_ = std::max(corner_quality_min_, corner_quality_);
  corner_quality_ = std::min(corner_quality_max_, corner_quality_);

  // update corner quality
  gftt_detector_->setQualityLevel(corner_quality_);
#endif

  // save features for the next iteration.
  prev_features_.clear();

  // Unpack keypoints and create regular features points
  for (auto&& key : features)
    prev_features_.push_back(key.pt);

  // save pyramids for the next iteration.
  last_pyramids_ = current_pyramids;

  // if few features were found, skip feature pairing on the next iteration
  if (prev_features_.size() < 10)
  {
    ROS_WARN_STREAM("(" << "#" << ") " << "few features found: " << prev_features_.size());
    first_image_ = true;
  }
}

// ----------------------------------------------------------------------------

void LKTTracker::set_max_features(int max_points, int max_)
{
#ifndef OPENCV_CUDA
  points_target_ = points_target;
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

}