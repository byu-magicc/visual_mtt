#include "visual_frontend/homography_calculator.h"

// GOAL:
// - provide a modular homography calculator that uses feature correspondences
// - this will be replaced by a homography filter when IMU data is used

HomographyCalculator::HomographyCalculator()
{

}


void HomographyCalculator::calculate_homography(std::vector<cv::Point2f>& prev_features,
                                                std::vector<cv::Point2f>& next_features)
{
  // use features to find homography
  // this will probably merge with the homography_filter class eventually


  // dummy method for compilation
  std::cout << "generating homography legit" << std::endl;

}
