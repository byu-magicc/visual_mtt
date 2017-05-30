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

  // use inlier count to determine if the homography is good
	int inlier_count = 0;
	for(int i = 0; i < inlier_mask_.size(); ++i)
	{
		inlier_count += inlier_mask_[i] ? 1 : 0;
	}
  if (inlier_count < 20)
  {
    std::cout << "Warning: Only " << inlier_count << " inliers; homography calculation may be inaccurate" << std::endl;
  }

  // Use the homography to transform the points forwards. This will put all of the
  // previous feature points on top of the next feature points except for points on
  // moving objects. Those will be off by some pixels, the pixel velocity.
  std::vector<cv::Point2f> corrected_pts;
  if (prev_features.size() > 0)
    cv::perspectiveTransform(prev_features, corrected_pts, homography_);

  // Find the point velocities
  // TODO these velocities are in pixels per frame: make this pixels per second.
  pixel_diff_.clear();
  for (int i = 0; i < corrected_pts.size(); ++i)
    pixel_diff_.push_back(next_features[i] - corrected_pts[i]);




  // NOTE: THIS NEEDS ADDITIONAL LOGIC TO CONSIDER THE CASE WHEN THERE ARE
  // FEW MATCHED FEATURES AND AN 'ELSE' HERE. WHAT HOMOGRAPHY SHOULD BE USED?

  // CERTAINLY WE SHOULD SET A BAD_HOMOGRAPHY FLAG SO THE MEASUREMENT SOURCES
  // THAT USE IT KNOW TO NOT GENERATE MEASUREMENTS
}
