#include "visual_frontend/visual_frontend.h"

namespace visual_frontend {

VisualFrontend::VisualFrontend()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh("~");

  // get parameters from param server that are not dynamically reconfigurable
  nh.param<bool>("tuning", tuning_, 0);

  if (tuning_)
    ROS_WARN("tuning mode enabled");

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_video  = it.subscribeCamera("video", 10, &VisualFrontend::callback_video,  this);
  sub_imu    = nh_.subscribe(     "imu",    1, &VisualFrontend::callback_imu,    this);
  sub_tracks = nh_.subscribe(     "tracks", 1, &VisualFrontend::callback_tracks, this);
  pub_scan   = nh_.advertise<visual_mtt::RRANSACScan>("measurements", 1);
  pub_stats  = nh_.advertise<visual_mtt::Stats>("stats", 1);

  // populate vector of desired measurement sources
  //sources_.push_back(std::shared_ptr<SourceBackground>(new SourceBackground()));
  sources_.push_back(std::shared_ptr<SourceFeatures>(new SourceFeatures()));

  // establish dynamic reconfigure and load defaults
  auto func = std::bind(&VisualFrontend::callback_reconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func);

  // temporary 0, until recognition is implemented
  t_recognition_ = ros::Duration(0);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
  // Only process every Nth frame
  static int frame = 0;
  if (frame++ % frame_stride_ != 0)
    return;

  tic_ = ros::Time::now();

  // calculate the frame delay
  // average overhead delay when the queue is empty is 2.4ms
  // warn if frame delay is greater than 50ms (ignoring first few frames)
  // ros::Duration delay = ros::Time::now() - data->header.stamp;
  // if (delay.toSec()>0.05 && frame>30)
  //   ROS_ERROR_STREAM("(" << frame << ") " << "visual frontend cannot run real-time: delay = " << delay.toSec() << " s");

  // save the camera parameters and frame timestamp
  timestamp_frame_ = data->header.stamp;

  // save camera parameters one time
  if (!info_received_)
  {
    // camera_matrix_ (K) is 3x3
    // dist_coeff_    (D) is a column vector of 4, 5, or 8 elements
    camera_matrix_ = cv::Mat(              3, 3, CV_64FC1);
    dist_coeff_    = cv::Mat(cinfo->D.size(), 1, CV_64FC1);

    // convert rosmsg vectors to cv::Mat
    for(int i=0; i<9; i++)
      camera_matrix_.at<double>(i/3, i%3) = cinfo->K[i];

    for(int i=0; i<cinfo->D.size(); i++)
      dist_coeff_.at<double>(i, 0) = cinfo->D[i];

    feature_manager_->camera_matrix_ = camera_matrix_;
    feature_manager_->dist_coeff_    = dist_coeff_;

    info_received_ = true;
  }

  // convert message data into OpenCV type cv::Mat
  hd_frame_in = cv_bridge::toCvCopy(data, "bgr8")->image;

  // downsize image to standard definition
  cv::Size def;
  def.width = hd_frame_in.cols*downsize_scale_;
  def.height = hd_frame_in.rows*downsize_scale_;
  cv::resize(hd_frame_in, sd_frame_in, def, 0, 0, cv::INTER_LINEAR);

  // add frames to recent history
  add_frame(hd_frame_in, hd_frame);
  add_frame(sd_frame_in, sd_frame);

<<<<<<< 8b44b5ef0bc5fbfda8c2bd1eb12d5ceace4c2391
  // find feature pairs (could be LK, NN, Brute Force)
  feature_manager_->find_correspondences(hd_frame); // in future operate on sd
=======
  // manage features (could be LK, NN, Brute Force)
  feature_manager_.find_correspondences(hd_frame); // in future operate on sd
>>>>>>> abstracting the HomographyManager
  toc_ = ros::Time::now();
  t_features_ = toc_ - tic_;

  // calculate the homography
  tic_ = ros::Time::now();
<<<<<<< 8b44b5ef0bc5fbfda8c2bd1eb12d5ceace4c2391
  homography_calculator_->calculate_homography(
    feature_manager_->prev_matched_,
    feature_manager_->next_matched_);
=======
  homography_manager_.calculate_homography(
    feature_manager_.prev_matched_,
    feature_manager_.next_matched_);
    // there is a reason *matched_ vectors are members of the subclass
    // it's for the future case with multiple FeatureManager/HomographyCalculator instantiations
>>>>>>> abstracting the HomographyManager
  toc_ = ros::Time::now();
  t_homography_ = toc_ - tic_;

  // call measurement sources execution
  tic_ = ros::Time::now();
  generate_measurements();
  toc_ = ros::Time::now();
  t_measurements_ = toc_ - tic_;

  // publish stats
  visual_mtt::Stats stats;
  stats.stride = frame_stride_;
  stats.times.push_back(t_features_.toSec());
  stats.times.push_back(t_homography_.toSec());
  stats.times.push_back(t_measurements_.toSec());
  stats.times.push_back(t_recognition_.toSec());
  pub_stats.publish(stats);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_imu(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
  // we expect to be getting data for this at >500 Hz (adjust queue for this)
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

void VisualFrontend::callback_tracks(const visual_mtt::TracksPtr& data)
{
  // save most recent track information in class (for use in measurement
  // sources such as direct methods)
  tracks_ = data;

  // call track_recognition bank (will use newest information and the high-res
  // video associated with the most recent update to maintain id descriptors.)
  // note: tracks updates will contain the original frame timestamp which
  // will be used to find the correct historical frame

  // recognition bank will host a function for checking new tracks with
  // existing id descriptors, this node will host a ROS service to interface
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_reconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level)
{
  // update: frontend, feature_manager_, homography_manager_, sources_
  set_parameters(config);
  feature_manager_.set_parameters(config);
  homography_manager_.set_parameters(config);

  for (int i=0; i<sources_.size(); i++)
    sources_[i]->set_parameters(config);

  ROS_INFO("visual frontend: parameters have been updated");
};

// ----------------------------------------------------------------------------

void VisualFrontend::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  frame_stride_ = config.frame_stride;
  downsize_scale_ = config.downsize_scale;
  // TODO: scale camera calibration for sd_image
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
  // optional delay for testing queue timing and warnings!
  // std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0/24)));

  // Message for publishing measurements to R-RANSAC Tracker
  visual_mtt::RRANSACScan scan;
  scan.header_frame.stamp = timestamp_frame_;
  if (!homography_manager_.homography_.empty())
    std::memcpy(&scan.homography, homography_manager_.homography_.data, scan.homography.size()*sizeof(float));


  for (int i=0; i<sources_.size(); i++)
  {
    sources_[i]->generate_measurements(
<<<<<<< 8b44b5ef0bc5fbfda8c2bd1eb12d5ceace4c2391
      homography_calculator_->homography_,
      feature_manager_->prev_matched_,
      feature_manager_->next_matched_,
      homography_calculator_->good_transform_);
=======
      homography_manager_.homography_,
      feature_manager_.next_matched_,
      homography_manager_.pixel_diff_,
      homography_manager_.good_transform_);
>>>>>>> abstracting the HomographyManager

    // when in tuning mode, display the measurements from each source
    // TODO: make pure virtual 'draw' function in source.h to keep this clean!
    if (tuning_)
    {
      // display measurements
      cv::Mat draw = hd_frame.clone();

      // treat points in the normalized image plane as 3D points (homogeneous).
      // project the points onto the sensor (pixel space) for plotting.
      // use no rotation or translation (world frame = camera frame).
      std::vector<cv::Point3f> features_h; // homogeneous
      std::vector<cv::Point2f> features_d; // distorted
      if (sources_[i]->features_.size()>0)
      {
        cv::convertPointsToHomogeneous(sources_[i]->features_, features_h);
        cv::projectPoints(features_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, features_d);
      }

      // plot measurements
      for (int j=0; j<features_d.size(); j++)
      {
        cv::Scalar color = cv::Scalar(255, 0, 255);
        cv::circle(draw, features_d[j], 2, color, 2, CV_AA);
      }
      cv::imshow(sources_[i]->name_, draw);
    }

    // Create a Source msg
    visual_mtt::Source src;
    src.id = i;
    src.dimensionality = 2; // TODO: Maybe ask the source what kind of measurements it produces?

    for (int j=0; j<sources_[i]->features_.size(); j++)
    {

      // TODO: Can I always assume that features_.size == features_vel_.size()?
      auto pos = sources_[i]->features_[j];
      auto vel = sources_[i]->features_vel_[j];

      visual_mtt::Measurement mpos, mvel;
      mpos.data = {pos.x, pos.y};
      mvel.data = {vel.x, vel.y};

      src.positions.push_back(mpos);
      src.velocities.push_back(mvel);
    }

    // Add source to scan message
    scan.sources.push_back(src);
  }

  if (tuning_)
  {
    // get the input from the keyboard
    char keyboard = cv::waitKey(1);
    if(keyboard == 'q')
      ros::shutdown();
  }

  // timestamp after scan is completed
  scan.header_scan.stamp = ros::Time::now();
  pub_scan.publish(scan);
}

}