#include "visual_frontend/visual_frontend.h"

namespace visual_frontend {

VisualFrontend::VisualFrontend()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh_private("~");

  // host the track recognition ROS service server
  srv_recognize_track_ = nh_private.advertiseService("recognize_track", &VisualFrontend::callback_srv_recognize_track, this);

  // connect the param update ROS service client
  srv_params_ = nh_.serviceClient<visual_mtt::RRANSACParams>("rransac/set_params");
  srv_params_.waitForExistence(ros::Duration(10.0));
  ROS_INFO("visual frontend: services successfully established, starting node");

  // get parameters from param server that are not dynamically reconfigurable
  nh_private.param<bool>("tuning", source_manager_.tuning_, 0);

  if (source_manager_.tuning_)
    ROS_WARN("tuning mode enabled");

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_video  = it.subscribeCamera("video", 10, &VisualFrontend::callback_video,  this);
  sub_tracks = nh_.subscribe(     "tracks", 1, &VisualFrontend::callback_tracks, this);
  pub_scan   = nh_.advertise<visual_mtt::RRANSACScan>("measurements", 1);
  pub_stats  = nh_.advertise<visual_mtt::Stats>("stats", 1);

  // establish dynamic reconfigure and load defaults (callback runs once here)
  auto func = std::bind(&VisualFrontend::callback_reconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
  // Only process every Nth frame
  static int frame = 0;
  if (frame++ % frame_stride_ != 0)
    return;

  // save the camera parameters and frame timestamp
  timestamp_frame_ = data->header.stamp;

  // convert message data into OpenCV type cv::Mat
  hd_frame_ = cv_bridge::toCvCopy(data, "bgr8")->image;

  // update the target recognition algorithm with the recent frame
  recognition_manager_.update_image(hd_frame_);

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

    // scale the entire matrix except the 3,3 element
    camera_matrix_scaled_ = camera_matrix_ * resize_scale_;
    camera_matrix_scaled_.at<double>(2,2) = 1;

    // set the high and low definition resolutions
    hd_res_.width = hd_frame_.cols;
    hd_res_.height = hd_frame_.rows;
    sd_res_.width = hd_frame_.cols*resize_scale_;
    sd_res_.height = hd_frame_.rows*resize_scale_;

    // provide algorithm members with updated camera parameters
    feature_manager_.set_camera(camera_matrix_scaled_, dist_coeff_, sd_res_);
    source_manager_.set_camera(camera_matrix_scaled_, dist_coeff_);
    recognition_manager_.set_camera(camera_matrix_, dist_coeff_);

    info_received_ = true;
  }

  //
  // Initial image processing
  //
  auto tic = ros::Time::now();

  // resize frame
#if OPENCV_CUDA
  cv::cuda::GpuMat hd_frame_cuda, sd_frame_cuda;
  hd_frame_cuda.upload(hd_frame_);
  cv::cuda::resize(hd_frame_cuda, sd_frame_cuda, sd_res_, 0, 0, cv::INTER_AREA);
  sd_frame_cuda.download(sd_frame_);
#else
  cv::resize(hd_frame_, sd_frame_, sd_res_, 0, 0, cv::INTER_AREA);
#endif

  //
  // Feature Manager: LKT Tracker, ORB-BN, etc
  //

  // manage features (could be LK, NN, Brute Force)
  feature_manager_.find_correspondences(sd_frame_);
  auto t_features = ros::Time::now() - tic;

  //
  // Homography Manager
  //
  tic = ros::Time::now();

  // calculate the homography
  homography_manager_.calculate_homography(feature_manager_.prev_matched_, feature_manager_.next_matched_);
  auto t_homography = ros::Time::now() - tic;

  //
  // Measurement Generation from multiple sources
  //
  tic = ros::Time::now();

  // call measurement sources execution
  source_manager_.generate_measurements(
    hd_frame_,
    sd_frame_,
    homography_manager_.homography_,
    feature_manager_.prev_matched_,
    feature_manager_.next_matched_,
    homography_manager_.good_transform_);

  // manage scan timestamps
  source_manager_.scan_.header_frame.stamp = timestamp_frame_;
  source_manager_.scan_.header_scan.stamp  = ros::Time::now();

  // copy the homography to the scan TODO: don't let the homography manager ever allow an empty matrix, then remove 'if'
  if (!homography_manager_.homography_.empty())
    std::memcpy(&source_manager_.scan_.homography, homography_manager_.homography_.data, source_manager_.scan_.homography.size()*sizeof(float));

  // publish scan
  pub_scan.publish(source_manager_.scan_);
  // TODO: move homography copy and timestamps into manager?

  auto t_measurements = ros::Time::now() - tic;

  //
  // publish stats
  //

  visual_mtt::Stats stats;
  stats.stride = frame_stride_;
  stats.t_feature_manager = t_features.toSec();
  stats.t_homography_manager = t_homography.toSec();
  stats.t_measurement_generation = t_measurements.toSec();
  pub_stats.publish(stats);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_tracks(const visual_mtt::TracksPtr& data)
{
  // rransac_node may be idle now, resend parameter update if needed
  if (srv_resend_)
  {
    // the last param update ROS service call failed, resend it
    if (srv_params_.call(srv_saved_))
    {
      // service call was successful, do not attempt to resend later
      srv_resend_ = false;
    }
    else
    {
      // service call was unsuccessful again
      ROS_WARN("failed to call R-RANSAC parameter service, resending");

      // attempt to resend later
      srv_resend_ = true;
    }
  }

  // save most recent track information in class (for use in measurement
  // sources such as direct methods) NOTE: not used (yet)!
  tracks_ = data;

  // use track information to update target descriptors
  recognition_manager_.update_descriptors(data);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_reconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level)
{
  // update: frontend, feature_manager_, homography_manager_, source_manager_
  set_parameters(config);
  feature_manager_.set_parameters(config);
  homography_manager_.set_parameters(config);
  source_manager_.set_parameters(config);
  recognition_manager_.set_parameters(config);

  // Update R-RANSAC specific parameters through a service call
  srv_set_params(config);

  ROS_INFO("visual frontend: parameters have been updated");
};

// ----------------------------------------------------------------------------

void VisualFrontend::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  frame_stride_ = config.frame_stride;
  resize_scale_ = config.resize_scale;

  // if camera information is saved, update the scaled camera parameters
  if (info_received_)
  {
    // scale the entire matrix except the 3,3 element
    camera_matrix_scaled_ = camera_matrix_ * resize_scale_;
    camera_matrix_scaled_.at<double>(2,2) = 1;

    // update the low definition resolution
    sd_res_.width = hd_res_.width*resize_scale_;
    sd_res_.height = hd_res_.height*resize_scale_;

    // provide algorithm members with updated camera parameters
    feature_manager_.set_camera(camera_matrix_scaled_, dist_coeff_, sd_res_);
    source_manager_.set_camera(camera_matrix_scaled_, dist_coeff_);
  }
}

// ----------------------------------------------------------------------------

bool VisualFrontend::callback_srv_recognize_track(visual_mtt::RecognizeTrack::Request &req, visual_mtt::RecognizeTrack::Response &res)
{
  // for debug and testing:
  // std::cout << "callback pinged in the frontend" << std::endl;

  res.id = recognition_manager_.identify_target(req.x, req.y);

  return true;
}

// ----------------------------------------------------------------------------

void VisualFrontend::srv_set_params(visual_mtt::visual_frontendConfig& config)
{
  visual_mtt::RRANSACParams srv;
  srv.request.published_video_scale = config.published_video_scale;
  srv.request.text_scale = config.text_scale;

  // retrieve the needed parameters for each source
  std::vector<uint32_t> id;
  std::vector<unsigned char> has_velocity;
  std::vector<double> sigmaR_pos;
  std::vector<double> sigmaR_vel;

  for (int i=0; i<source_manager_.n_sources_; i++)
  {
    id.push_back(source_manager_.measurement_sources_[i]->id_);
    has_velocity.push_back(source_manager_.measurement_sources_[i]->has_velocity_);
    sigmaR_pos.push_back(source_manager_.measurement_sources_[i]->sigmaR_pos_);
    sigmaR_vel.push_back(source_manager_.measurement_sources_[i]->sigmaR_vel_);
  }

  srv.request.n_sources    = source_manager_.n_sources_;
  srv.request.id           = id;
  srv.request.has_velocity = has_velocity;
  srv.request.sigmaR_pos   = sigmaR_pos;
  srv.request.sigmaR_vel   = sigmaR_vel;

  // save a copy of the service request object
  srv_saved_ = srv;

  if (srv_params_.call(srv))
  {
    // service call was successful, do not attempt to resend later
    srv_resend_ = false;
  }
  else
  {
    // service call was unsuccessful
    ROS_INFO("failed to call R-RANSAC parameter service, resending");

    // attempt to resend later
    srv_resend_ = true;
  }
}

}
