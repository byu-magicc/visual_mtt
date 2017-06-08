#include "visual_frontend/source_features.h"

namespace visual_mtt {

SourceFeatures::SourceFeatures()
{

  name_ = "Homography Outliers";

}

// ----------------------------------------------------------------------------

void SourceFeatures::generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& features, std::vector<cv::Point2f>& features_vel, bool good_transform)
{

  if (!first_image_)
  {
    // Warp previous image to current image frame
    // warpPerspective(previous_image_, mask, homography, mono.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar());

    // Use absolute difference to make a point mask
    // absdiff(mono, mask, mask);
    // mask_new_area(mask, homography);

    // Apply filter to get a better mask
    // GaussianBlur(mask, mask, gauss_filter_size_, GAUSSIAN_BLUR_SIGMA, GAUSSIAN_BLUR_SIGMA);
    // morphologyEx(mask, mask, MORPH_CLOSE, morph_element_);

    // Threshold and perform operations on the difference image to make a good mask
    // threshold(mask, mask, MASK_THOLD_, 255, THRESH_BINARY);

    // Identify moving features based on the point mask
    // Also remove moving points that do not fit the velocity filters (in pixels per frame at the moment)
    std::vector<cv::Point2f> features_filtered;
    std::vector<cv::Point2f> features_vel_filtered;
    int numberOfPossibleMovers = 0;
    for (int ii = 0; ii < features.size(); ++ii)
    {
      float vel = sqrt(features_vel[ii].x*features_vel[ii].x + features_vel[ii].y*features_vel[ii].y);
      // if (mask.at<uchar>(features[ii].y, features[ii].x) == 255)
      // {
        if (vel > velocity_floor_ && vel < velocity_ceiling_)
        {
          numberOfPossibleMovers++;
          features_filtered.push_back(features[ii]);
          features_vel_filtered.push_back(features_vel[ii]);
        }

      // }
    }
    if (features.size() > 0)
    {
      // if a high proportion of features are outliers, print a warning
      if ((double)numberOfPossibleMovers / (double)features.size() > homography_error_thold_)
      {
        std::cout << "many homography outliers: may be the result of a bad homography alignment." << std::endl; // create a proper warning message
      }
    }
    features_ = features_filtered;
    features_vel_ = features_vel_filtered;
  }
  else
  {
    first_image_ = false;
    // Generate (white) image for a new area mask
    // mask_frame_ = Mat(mono.rows, mono.cols, CV_8U, 255);
    // mask = Mat::zeros(mono.rows, mono.cols, CV_8UC1);
  }



}

// ----------------------------------------------------------------------------

void SourceFeatures::set_parameters(visual_mtt2::visual_frontendConfig& config)
{

  velocity_floor_ = config.minimum_pixel_velocity;
  velocity_ceiling_ = config.maximum_pixel_velocity;

}

} // namespace visual_mtt