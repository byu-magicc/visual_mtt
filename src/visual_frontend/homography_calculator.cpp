#include "visual_frontend/homography_calculator.h"

// GOAL:
// - provide a modular homography calculator that uses feature correspondences
// - this will be replaced by a homography filter when IMU data is used

HomographyCalculator::HomographyCalculator()
{

}


void HomographyCalculator::calculate_homography(std::vector<cv::Point2f>& prev_features,
                                                std::vector<cv::Point2f>& features)
{

  // use features to find homography
  // why does this need its own class? so that it can be called whenever needed
  // independent of the feature management, for modularity, and the feature
  // management is very cluttered.

}