#include "visual_frontend/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)

namespace visual_mtt {

FeatureManager::FeatureManager(ros::NodeHandle nh)
{
  first_image_ = true;

  // get the needed params that are not dynamically reconfigurable
  int pyramid_size;
  nh.param<double>("visual_frontend/corner_quality",       corner_quality_,       0.03 );
  nh.param<double>("visual_frontend/corner_quality_min",   corner_quality_min_,   0.03 );
  nh.param<double>("visual_frontend/corner_quality_max",   corner_quality_max_,   0.05 );
  nh.param<double>("visual_frontend/corner_quality_alpha", corner_quality_alpha_, 0.999);
  nh.param<int>   ("visual_frontend/pyramid_size",         pyramid_size,          21   );

  pyramid_size_ = cv::Size(pyramid_size, pyramid_size);

  double minDistance=10;           // TODO use rosparam? probably not worth dynamically reconfigurable.
  int blockSize=3;                 // TODO use rosparam? probably not worth dynamically reconfigurable.
  bool useHarrisDetector=false;    // just hardcode the false in?
  // double k=0.04;                // opencv default is 0.04, plus this isn't needed if above is false

  gftt_detector_ = cv::GFTTDetector::create(points_max_, corner_quality_, minDistance, blockSize, useHarrisDetector);

  kltTerm_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.03);
  // TODO: use rosparam?

  // eventually: make a "feature_detector_" that can be set to use various
  // feature appraoches (gftt, orb, etc)
  // maybe: have a "feature_matcher_" that can be set to use various
  // matching appraoches (lk, etc)
}

// ----------------------------------------------------------------------------

void FeatureManager::set_parameters(visual_mtt2::visual_frontendConfig& config)
{

  points_max_ = config.points_max;
  points_target_ = config.points_target;
  gftt_detector_->setMaxFeatures(points_max_);

}

// ----------------------------------------------------------------------------

void FeatureManager::find_correspondences(cv::Mat& img)
{
  // std::cout << "generating feature correspondences" << std::endl;
  // FOR NOW: GENERATE BASIC FEATURE CORRESPONDENCES USING LK
  // THE FOLLOWING IS MOSTLY FROM ORIGINAL CODE
  // but omitting the orientation-based guess and adjusting inputs

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
    prev_matched_.clear();
    next_matched_.clear();
    for(unsigned int ii = 0; ii < status.size(); ++ii)
    {
      if (status[ii])
      {
        prev_matched_.push_back(prev_features_[ii]);
        next_matched_.push_back(next_features[ii]);
      }
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

  // perform adaptive corner quality using discrete alpha filtering
  // first determine the direction based on the number of features found
  int quality_step_dir = 0;
  if (features.size() < points_target_)
  {
    quality_step_dir = -1;
  }
  else
  {
    quality_step_dir = 1;
  }
  // apply alpha filter and upper/lower bounds
  corner_quality_ = corner_quality_*corner_quality_alpha_ + quality_step_dir*(1-corner_quality_alpha_);
  corner_quality_ = std::max(corner_quality_min_, corner_quality_);
  corner_quality_ = std::min(corner_quality_max_, corner_quality_);

  // update corner quality
  gftt_detector_->setQualityLevel(corner_quality_);
  // std::cout << "New quality: " << gftt_detector_->getQualityLevel() << std::endl;

  // save features for the next iteration.
  prev_features_.clear();
  keyPointVecToPoint2f(features, prev_features_);

  // save pyramids for the next iteration.
  last_pyramids_ = current_pyramids;

  // if few features were found, skip feature pairing on the next iteration
  if (prev_features_.size() < 10)
  {
    std::cout << "few features found in this frame." << std::endl; // create a proper warning message
    first_image_ = true;
  }
}

// ----------------------------------------------------------------------------

void FeatureManager::keyPointVecToPoint2f(std::vector<cv::KeyPoint>& keys, std::vector<cv::Point2f>& pts)
{
  // TODO rename this method
  for (int i = 0; i < keys.size(); i++)
  {
    pts.push_back(keys[i].pt);
  }
}

} // namespace visual_mtt