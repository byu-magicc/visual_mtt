#include "visual_frontend/source_manager/difference_image.h"

namespace visual_frontend {

DifferenceImage::DifferenceImage()
{
  name_ = "Difference Image";
}

// ----------------------------------------------------------------------------

DifferenceImage::~DifferenceImage()
{
  cv::destroyWindow(name_);
}

// ----------------------------------------------------------------------------

void DifferenceImage::generate_measurements(cv::Mat& hd_frame, cv::Mat& sd_frame, cv::Mat& homography, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& next_features, bool good_transform)
{
  // save the original image for plotting
  sd_frame_ = sd_frame;

  // undisort the low resolution image
  cv::undistort(sd_frame_, frame_u_, camera_matrix_, dist_coeff_);

  if (!first_image_)
  {
    // convert the current euclidean homography to pixel homography
    camera_matrix_.convertTo(camera_matrix_, CV_32FC1); // for inverse
    cv::Mat homography2 = camera_matrix_*homography*camera_matrix_.inv();

    // transform previous image using new homography
    cv::warpPerspective(frame_u_last_, frame_u_last_, homography2, frame_u_.size());

    // difference
    cv::Mat diff;
    cv::absdiff(frame_u_, frame_u_last_, diff);
    cv::imshow("(1) difference", diff);

    // mask the artifact edges TODO: move to separate function

    // get usable corners
    std::vector<cv::Point2f> corners; // TODO: generate at beginning, dyn_recon
    double border = 5; // TODO: make hard-coded
    corners.push_back(cv::Point2f(border,               border));
    corners.push_back(cv::Point2f(frame_u_.cols-border, border));
    corners.push_back(cv::Point2f(frame_u_.cols-border, frame_u_.rows-border));
    corners.push_back(cv::Point2f(border,               frame_u_.rows-border));

    // see where the old corners land in current frame
    cv::Mat corners_warped;
    cv::perspectiveTransform(corners, corners_warped, homography2);
    corners_warped.convertTo(corners_warped, CV_32SC1);

    // generate mask
    cv::Mat mask(frame_u_.size(), CV_8UC1, cv::Scalar(0));
    cv::fillConvexPoly(mask, corners_warped, cv::Scalar(255));
    // cv::imshow("\"what to keep from raw diff\" mask", mask);

    // apply mask
    cv::cvtColor(diff, diff, CV_BGR2GRAY);
    cv::bitwise_and(diff, mask, diff);
    // cv::imshow("(1) difference", diff);

    // blur
    cv::GaussianBlur(diff, diff, ksize_, sigma_);
    cv::imshow("(2) blur", diff);

    // normalize
    cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
    cv::imshow("(3) normalize", diff);

    // threshold
    cv::threshold(diff, diff, threshold_, 255, cv::THRESH_BINARY);
    cv::imshow("(4) threshold", diff);

    // open operation (erode then dilate)
    cv::morphologyEx(diff, diff, cv::MORPH_OPEN, element_, cv::Point(-1,-1), morph_iterations_);
    cv::imshow("(5) open", diff);

    // point generation


    // turn the edges into many series of points
    std::vector<std::vector<cv::Point>> contours1, contours2;
    cv::findContours(diff, contours1, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    cv::Mat outlines1(diff.size(), CV_64FC3, cv::Scalar(0, 0, 0));
    cv::drawContours(outlines1, contours1, -1, cv::Scalar(0,255,0));
    cv::imshow("(6) all contours", outlines1);

    // filter difference outlines based on contour complexity
    for (int i; i<contours1.size(); i++)
    {
      int complexity = contours1[i].size(); // switch to distance along contour
      if (complexity > minimum_diff_complexity_ && complexity < maximum_diff_complexity_)
      {
        contours2.push_back(contours1[i]);
      }
    }
    cv::Mat outlines2(diff.size(), CV_64FC3, cv::Scalar(0, 0, 0));
    cv::drawContours(outlines2, contours2, -1, cv::Scalar(255,0,255));
    cv::imshow("(7) filtered contours", outlines2);

    // find centroids of remaining outlines






    // TODO: good_transform flag not yet used! (if false, discard measurements)
    // replace first_image_ logic with good_transform logic and make sure
    // it is set to 'false' in the homography manager on first iteration!
  }
  else
  {
    first_image_ = false;
  }

  // bump undistorted image (save, overwriting the old one)
  frame_u_last_ = frame_u_.clone();

}

// ----------------------------------------------------------------------------

void DifferenceImage::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // example parameter set
  ksize_ = cv::Size(2*config.ksize + 1, 2*config.ksize + 1);
  sigma_ = config.sigma;
  element_ = cv::getStructuringElement(
    cv::MORPH_RECT,
    cv::Size(2*config.morph_size + 1, 2*config.morph_size+1),
    cv::Point(config.morph_size, config.morph_size));
  morph_iterations_ = config.morph_iterations;
  threshold_ = config.threshold;
  minimum_diff_complexity_ = config.minimum_diff_complexity;
  maximum_diff_complexity_ = config.maximum_diff_complexity;
}

// ----------------------------------------------------------------------------

void DifferenceImage::set_camera(const cv::Mat& K, const cv::Mat& D)
{
  camera_matrix_ = K.clone();
  dist_coeff_ = D.clone();
}

// ----------------------------------------------------------------------------

void DifferenceImage::draw_measurements()
{
  cv::Mat draw = sd_frame_.clone();

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> features_h; // homogeneous
  std::vector<cv::Point2f> features_d; // distorted
  if (features_.size()>0)
  {
    cv::convertPointsToHomogeneous(features_, features_h);
    cv::projectPoints(features_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, features_d);
  }

  // plot measurements
  for (int j=0; j<features_d.size(); j++)
  {
    cv::Scalar color = cv::Scalar(255, 0, 255); // TODO: set before loop, or make member
    cv::circle(draw, features_d[j], 2, color, 2, CV_AA);
  }
  cv::imshow(name_, draw);

  // get the input from the keyboard, force display
  char keyboard = cv::waitKey(1);
  if(keyboard == 'q')
    ros::shutdown();

}

}