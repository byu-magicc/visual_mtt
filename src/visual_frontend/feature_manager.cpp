#include "visual_frontend/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)


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


  //int maxCorners=points_max_;
  // double qualityLevel=corner_quality_;
  double minDistance=10;           // TODO parameterize this
  int blockSize=3;                 // TODO parameterize this
  bool useHarrisDetector=false;    // just hardcode the false in?
  double k=0.04;                   // look this up

#if CV_MAJOR_VERSION == 2
  // we won't be supporting opencv2
  gftt_detector_ = cv::Ptr<cv::GoodFeaturesToTrackDetector>(new cv::GoodFeaturesToTrackDetector(
                              points_max_, corner_quality_, minDistance, blockSize, useHarrisDetector, k));
  //grid_detector_ = Ptr<FeatureDetector>(new GridAdaptedFeatureDetector(
  //                                                  detector_, points_max_, 6, 8));

#elif CV_MAJOR_VERSION == 3
  gftt_detector_ = cv::GFTTDetector::create(points_max_, corner_quality_, minDistance, blockSize, useHarrisDetector, k);
#endif

  kltTerm_ = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.03);

  // eventually: make gftt_detector_ a generic feature generator that
  // has the ability to use various feature appraoches (gftt, orb, etc)

}

// ----------------------------------------------------------------------------

void FeatureManager::set_parameters(visual_mtt2::visual_frontendConfig& config)
{
  std::cout << "feature_manager update" << std::endl; // temporary

  points_max_ = config.points_max;
  points_target_ = config.points_target;

  gftt_detector_->setMaxFeatures(points_max_);

}

// ----------------------------------------------------------------------------

void FeatureManager::find_correspondences(cv::Mat& img)
{
  std::cout << "generating feature correspondences" << std::endl;
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
    std::vector<unsigned char> status_nopredict;
    std::vector<float> err_nopredict;

    // match feature points using lk algorithm
    cv::calcOpticalFlowPyrLK(last_pyramids_, current_pyramids,
                             prev_features_, next_features,
                             status, err, pyramid_size_, 3, kltTerm_, 0, 1e-4);

    // store only matched features
    prev_matched_.clear();
    next_matched_.clear();
    // std::vector<cv::Point2f> prev_matched_nopredict;
    // std::vector<cv::Point2f> next_matched_nopredict;
    //selectMatchedPoints(status, prev_features_, next_features, prev_matched, next_matched);
    for(unsigned int ii = 0; ii < status.size(); ++ii)
    {
      if (status[ii])
      {
        prev_matched_.push_back(prev_features_[ii]);
        next_matched_.push_back(next_features[ii]);
      }
    }

    // the homography calculator will perform a check on the number of matches

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

#if CV_MAJOR_VERSION == 2
  gftt_detector_->setDouble("qualityLevel",corner_quality_);
  //cout << 1.0, "New quality: " << gftt_detector_->getDouble("qualityLevel") << endl;
#elif CV_MAJOR_VERSION == 3
  gftt_detector_->setQualityLevel(corner_quality_);
  //cout << 1.0, "New quality: " << gftt_detector_->getQualityLevel() << endl;
#endif

  // save features for the next iteration.
  prev_features_.clear();
  keyPointVecToPoint2f(features, prev_features_);

  // save image for the next iteration.
  last_pyramids_ = current_pyramids;
  //last_image_ = mono.clone(); // TODO not used anywhere, double check this then remove it

  // if few features were found, skip feature pairing on the next iteration
  if (prev_features_.size() < 10)
  {
    first_image_ = true;
    std::cout << "few features found in this frame." << std::endl;
    // set a flag so homography calculator knows TODO
  }
}

// ----------------------------------------------------------------------------

void FeatureManager::keyPointVecToPoint2f(std::vector<cv::KeyPoint>& keys, std::vector<cv::Point2f>& pts)
{
  for (int i = 0; i < keys.size(); i++)
  {
    pts.push_back(keys[i].pt);
  }
}

