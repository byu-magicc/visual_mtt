#include "visual_frontend/homography_calculator.h"

// GOAL:
// - provide a modular homography calculator that uses feature correspondences
// - this will be replaced by a homography filter when IMU data is used

HomographyCalculator::HomographyCalculator()
{
  // this will probably merge with the homography_filter class eventually
  // into a plain "homography" class, we'll see.
}

// ----------------------------------------------------------------------------

void HomographyCalculator::set_parameters(visual_mtt2::visual_frontendConfig& config)
{
  std::cout << "homography_calculator update" << std::endl; // temporary
  // add other param updates here
}

// ----------------------------------------------------------------------------

void HomographyCalculator::calculate_homography(const std::vector<cv::Point2f>& prev_features,
                                                const std::vector<cv::Point2f>& next_features)
{
  std::cout << "generating homography" << std::endl;
  // use features to find homography

  int ransac_reproj_threshold = 1; // TODO: parameterize this

  if (prev_features.size() > 4)
  {
    // calculate the homography
    homography_ = cv::findHomography(prev_features, next_features,
      CV_RANSAC, ransac_reproj_threshold, inlier_mask_);

    // baptize the homography
    homography_.convertTo(homography_, CV_32F);
  }

  // NOTE: THIS NEEDS ADDITIONAL LOGIC TO CONSIDER THE CASE WHEN THERE ARE
  // FEW MATCHED FEATURES AND AN ELSE HERE. WHAT HOMOGRAPHY SHOULD BE USED?

  // CERTAINLY WE SHOULD SET A BAD_HOMOGRAPHY FLAG SO THE MEASUREMENT SOURCES
  // THAT USE IT KNOW TO NOT GENERATE MEASUREMENTS
}
