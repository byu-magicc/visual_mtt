#include "visual_frontend/visual_frontend.h"

namespace visual_mtt {

VisualFrontend::VisualFrontend()
{
  // get parameters from param server that are not dynamically reconfigurable
  double fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6;
  nh.param<double>("visual_frontend/calibration/fx", fx, 0);
  nh.param<double>("visual_frontend/calibration/fy", fy, 0);
  nh.param<double>("visual_frontend/calibration/cx", cx, 0);
  nh.param<double>("visual_frontend/calibration/cy", cy, 0);
  nh.param<double>("visual_frontend/calibration/k1", k1, 0);
  nh.param<double>("visual_frontend/calibration/k2", k2, 0);
  nh.param<double>("visual_frontend/calibration/p1", p1, 0);
  nh.param<double>("visual_frontend/calibration/p2", p2, 0);
  nh.param<double>("visual_frontend/calibration/k3", k3, 0);
  nh.param<double>("visual_frontend/calibration/k4", k4, 0);
  nh.param<double>("visual_frontend/calibration/k5", k5, 0);
  nh.param<double>("visual_frontend/calibration/k6", k6, 0);

  calibration_ = (cv::Mat_<float>(3,3) <<
    fx ,  0.0,  cx,
    0.0,  fy ,  cy,
    0.0,  0.0,  1.0);
  distortion_ = (cv::Mat_<float>(8,1) << k1, k2, p1, p2, k3, k4, k5, k6);

  // ROS communication
  sub_video  = nh.subscribe("video",  1, &VisualFrontend::callback_video,  this);
  sub_imu    = nh.subscribe("imu",    1, &VisualFrontend::callback_imu,    this);
  sub_tracks = nh.subscribe("tracks", 1, &VisualFrontend::callback_tracks, this);
  pub        = nh.advertise<visual_mtt2::RRANSACScan>("measurements", 1);

  // key member objects
  feature_manager_       = std::shared_ptr<FeatureManager>(new FeatureManager(nh));
  homography_calculator_ = std::shared_ptr<HomographyCalculator>(new HomographyCalculator());

  // populate vector of desired measurement sources
  //sources_.push_back(std::shared_ptr<SourceBackground>(new SourceBackground()));
  sources_.push_back(std::shared_ptr<SourceFeatures>(new SourceFeatures()));

  // establish dynamic reconfigure and load defaults
  auto func = std::bind(&VisualFrontend::callback_reconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_video(const sensor_msgs::ImageConstPtr& data)
{
  // future work TODO:
  // decimation logic (Nth frame)
  // resize frame to lower resolution (keep both)
    // a short history is needed for the low res for the sliding
    // a short history is needed for the high res so the track recognition can
    // locate the high-res frame associated with the track it's subscribing to


  // generate frame timestamp
  frame_timestamp_ = ros::Time::now();

  // convert message data into OpenCV type cv::Mat
  hd_frame_in = cv_bridge::toCvCopy(data, "bgr8")->image;

  // downsize image to standard definition
  cv::resize(hd_frame_in, sd_frame_in, sd_frame_in.size(), 0, 0, cv::INTER_LINEAR);

  // add frames to recent history
  add_frame(hd_frame_in, hd_frame);
  add_frame(sd_frame_in, sd_frame);

  // manage features (could be LK, NN, Brute Force)
  feature_manager_->find_correspondences(hd_frame); // in future operate on sd

  // consider if IMU is ignored (param from launchfile)
  // if IMU     ignored, call homography_calculator (feature correspondences)
  // if IMU not ignored, call the homography_filter filter update
  homography_calculator_->calculate_homography(
    feature_manager_->prev_matched_,
    feature_manager_->next_matched_);
    // there is a reason *matched_ vectors are members of the subclass
    // it's for the future case with multiple FeatureManager/HomographyCalculator instantiations

  // call measurement sources execution
    // (use updated recent images)
    // (use already-generated feature correpsondences)
    // (use already-generated homography)
    // (use updated recent track data)
  generate_measurements();

  // publish measurements and homography


  // TODO: create a display function that considers whether tuning=true
  // display hd and sd frames
  cv::imshow("hd image", hd_frame);
  cv::imshow("sd image", sd_frame);
  // get the input from the keyboard
  char keyboard = cv::waitKey(10);
  if(keyboard == 'q')
    ros::shutdown();
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_imu(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
  // we expect to be getting data for this at >500 Hz
  //
  // for in-frame tracking, we'll need 2 homographies:
  // one between the sliding frames for background subtraction
  // one between the two recent frames for updating history and estimates
  // if the homography filter operates on a manifold, maybe it can help?
  // NOTE: need better understanding of homography filter to answer this.
  // ---------------------------------------------------

  // if IMU not ignored (from launchfile), call homography filter IMU propagate
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_tracks(const visual_mtt2::TracksPtr& data)
{
  // save most recent track information in class (for use in measurement
  // sources such as direct methods)

  // call track_recognition bank (will use newest information and the high-res
  // video associated with the most recent update to maintain id descriptors.)
  // note: tracks updates will contain the original frame timestamp which
  // will be used to find the correct historical frame

  // recognition bank will host a function for checking new tracks with
  // existing id descriptors, this node will host a ROS service to interface

}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_reconfigure(visual_mtt2::visual_frontendConfig& config, uint32_t level)
{
  // update parameters:
  // VisualFrontend, FeatureManager, HomographyCalculator, Sources
  set_parameters(config);
  feature_manager_->set_parameters(config);
  homography_calculator_->set_parameters(config);

  for (int i=0; i<sources_.size(); i++)
  {
    sources_[i]->set_parameters(config);
  }
};

// ----------------------------------------------------------------------------

void VisualFrontend::set_parameters(visual_mtt2::visual_frontendConfig& config)
{
  std::cout << "frontend update" << std::endl; // temporary
  // add other param updates here
}


// ----------------------------------------------------------------------------

void VisualFrontend::add_frame(cv::Mat& newMat, cv::Mat& memberMat) // second argument: uMat
{

  // why does this need its own method?
  // see https://github.com/jdmillard/opencv-cuda
  memberMat = newMat;

}

// ----------------------------------------------------------------------------

void VisualFrontend::generate_measurements()
{
  // Message for publishing measurements to R-RANSAC Tracker
  visual_mtt2::RRANSACScan scan;
  scan.header.stamp = ros::Time::now();
  if (!homography_calculator_->homography_.empty())
    memcpy(&scan.homography, homography_calculator_->homography_.data, scan.homography.size()*sizeof(float));

  for (int i=0; i<sources_.size(); i++)
  {
    sources_[i]->generate_measurements(
      homography_calculator_->homography_,
      feature_manager_->next_matched_,
      homography_calculator_->pixel_diff_,
      homography_calculator_->good_transform_);

    // Create a Source msg
    visual_mtt2::Source src;
    src.id = i;
    src.dimensionality = 2; // TODO: Maybe ask the source what kind of measurements it produces?

    for (int j=0; j<sources_[i]->features_.size(); j++)
    {

      // TODO: Can I always assume that features_.size == features_vel_.size()?
      auto pos = sources_[i]->features_[j];
      auto vel = sources_[i]->features_vel_[j];

      visual_mtt2::Measurement mpos, mvel;
      mpos.data = {pos.x, pos.y};
      mvel.data = {vel.x, vel.y};

      src.positions.push_back(mpos);
      src.velocities.push_back(mvel);
    }



    // Add source to scan message
    scan.sources.push_back(src);
  }

  pub.publish(scan);

  // each source will recieve (and can ignore or use):
    // recent images
    // feature correpsondences
    // homography
    // recent track data






  // HACKED, TEMPORARY FOR TESTING !!!!
  // display measurements
  cv::Mat draw = hd_frame.clone();
  // plot measurements
  for (int jj=0; jj<sources_[0]->features_.size(); jj++)
  {
    cv::Scalar color = cv::Scalar(255, 0, 255);
    //std::cout << "filtered point" << std::endl;
    cv::circle(draw, sources_[0]->features_[jj], 2, color, 2, CV_AA);
  }

  cv::imshow("homography outlier measurements", draw);
  // get the input from the keyboard
  char keyboard = cv::waitKey(10);
  if(keyboard == 'q')
    ros::shutdown();

  // see source_measurement.h for question about measurement structure
}

}
