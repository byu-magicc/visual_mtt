#include "visual_frontend/feature_manager.h"

// GOAL:
// - provide a feature manager that finds feature correspondences
// - provide options to use different feature types (ORB, GFTT)
// - provide options to use different feature matching methods (LK, NN, BF)

FeatureManager::FeatureManager()
{

  // come up with a nice param setup (follow rransac repo)

}

void FeatureManager::find_correspondences()
{
  // dummy method for compilation
  std::cout << "generating feature correspondences" << std::endl;
}
void FeatureManager::find_correspondences(cv::Mat& prev_frame, cv::Mat& frame,
                                          std::vector<cv::Point2f>& prev_features,
                                          std::vector<cv::Point2f>& features,
                                          std::vector<cv::Point2f>& diff_features,
                                          cv::Mat& drawing, bool draw)
{

  // use provided frames and previous features to find feature correspondences
  // using whatever matching method has been chosen

  // FOR NOW: GENERATE BASIC FEATURE CORRESPONDENCES USING LK


}




