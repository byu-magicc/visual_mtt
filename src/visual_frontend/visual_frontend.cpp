#include "visual_frontend/visual_frontend.h"

namespace visual_frontend {

VisualFrontend::VisualFrontend()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh_private("~");

  // // host the track recognition ROS service server
  // srv_recognize_track_ = nh_private.advertiseService("recognize_track", &VisualFrontend::callback_srv_recognize_track, this);

  // get parameters from param server that are not dynamically reconfigurable
  nh_private.param<bool>("tuning", source_manager_.tuning_, 0);

  if (source_manager_.tuning_)
    ROS_WARN("tuning mode enabled");

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_video  = it.subscribeCamera("video", 10, &VisualFrontend::callback_video,  this);
  // sub_tracks = nh_.subscribe(     "tracks", 1, &VisualFrontend::callback_tracks, this);
  // pub_scan   = nh_.advertise<visual_mtt::RRANSACScan>("measurements", 1);
  // pub_stats  = nh_.advertise<visual_mtt::Stats>("stats", 1);

  pub_tracks       = nh_.advertise<visual_mtt::Tracks>("tracks", 1);
  pub_tracks_video = it.advertise("tracks_video", 1);

  // populate plotting colors
  colors_ = std::vector<cv::Scalar>();
  for (int i = 0; i < 1000; i++)
    colors_.push_back(cv::Scalar(std::rand() % 256, std::rand() % 256, std::rand() % 256));

  // establish dynamic reconfigure and load defaults (callback runs once here)
  auto func1 = std::bind(&VisualFrontend::callback_reconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func1);

  // establish dynamic reconfigure and load defaults (callback runs once here)
  ros::NodeHandle nh_private_rransac("rransac");
  rransac_server_.reset(new dynamic_reconfigure::Server<visual_mtt::rransacConfig>(nh_private_rransac));
  auto func2 = std::bind(&VisualFrontend::callback_reconfigure_rransac, this, std::placeholders::_1, std::placeholders::_2);
  rransac_server_->setCallback(func2);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
  // Only process every Nth frame
  if (frame_++ % frame_stride_ != 0)
    return;

  // save the camera parameters and frame timestamp
  timestamp_frame_ = data->header.stamp;

  // convert message data into OpenCV type cv::Mat
  hd_frame_ = cv_bridge::toCvCopy(data, "bgr8")->image;

  // update the target recognition algorithm with the recent frame
  // recognition_manager_.update_image(hd_frame_);

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


    // update surveillance region based on normalized image plane
    // corner_.push_back(cv::Point2f(0,0));
    cv::undistortPoints(corner_, corner_, camera_matrix_, dist_coeff_);

    double percentage;
    nh_.param<double>("rransac/surveillance_region", percentage, 0);

    params_.field_max_x = std::abs(corner_[0].x*percentage);
    params_.field_max_y = std::abs(corner_[0].y*percentage);
    tracker_.set_parameters(params_);



    info_received_ = true;
  }

  //
  // Initial image processing
  //
  // auto tic = ros::Time::now();

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
  // auto t_features = ros::Time::now() - tic;

  //
  // Homography Manager
  //
  // tic = ros::Time::now();

  // calculate the homography
  homography_manager_.calculate_homography(feature_manager_.prev_matched_, feature_manager_.next_matched_);
  // auto t_homography = ros::Time::now() - tic;

  //
  // Measurement Generation from multiple sources
  //
  // tic = ros::Time::now();

  // call measurement sources execution
  source_manager_.feed_rransac(tracker_,
    hd_frame_, sd_frame_,
    homography_manager_.homography_,
    homography_manager_.good_transform_,
    feature_manager_.prev_matched_,
    feature_manager_.next_matched_);


  //
  // R-RANSAC Tracker
  //

  Eigen::Matrix3f HH;
  cv::cv2eigen(homography_manager_.homography_, HH);
  Eigen::Projective2d T(HH.cast<double>());
  tracker_.apply_transformation(T);

  // measurements have already been added by source manager

  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  std::vector<rransac::core::ModelPtr> tracks = tracker_.run();

  // publish the tracks onto ROS network
  publish_tracks(tracks);

  // generate visualization only, but if someone is listening
  if (pub_tracks_video.getNumSubscribers() > 0)
    draw_tracks(tracks);


  // manage scan timestamps
  // source_manager_.scan_.header_frame.stamp = timestamp_frame_;
  // source_manager_.scan_.header_scan.stamp  = ros::Time::now();

  // copy the homography to the scan TODO: don't let the homography manager ever allow an empty matrix, then remove 'if'
  // if (!homography_manager_.homography_.empty())
  //   std::memcpy(&source_manager_.scan_.homography, homography_manager_.homography_.data, source_manager_.scan_.homography.size()*sizeof(float));

  // publish scan
  // pub_scan.publish(source_manager_.scan_);
  // TODO: move homography copy and timestamps into manager?

  // auto t_measurements = ros::Time::now() - tic;

  //
  // publish stats
  //

  // visual_mtt::Stats stats;
  // stats.stride = frame_stride_;
  // stats.t_feature_manager = t_features.toSec();
  // stats.t_homography_manager = t_homography.toSec();
  // stats.t_measurement_generation = t_measurements.toSec();
  // pub_stats.publish(stats);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_tracks(const visual_mtt::TracksPtr& data)
{
  // // rransac_node may be idle now, resend parameter update if needed
  // if (srv_resend_)
  // {
  //   // the last param update ROS service call failed, resend it
  //   if (srv_params_.call(srv_saved_))
  //   {
  //     // service call was successful, do not attempt to resend later
  //     srv_resend_ = false;
  //   }
  //   else
  //   {
  //     // service call was unsuccessful again
  //     ROS_WARN("failed to call R-RANSAC parameter service, resending");

  //     // attempt to resend later
  //     srv_resend_ = true;
  //   }
  // }

  // // save most recent track information in class (for use in measurement
  // // sources such as direct methods) NOTE: not used (yet)!
  // tracks_ = data;

  // // use track information to update target descriptors
  // recognition_manager_.update_descriptors(data);
}

// ----------------------------------------------------------------------------

void VisualFrontend::callback_reconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level)
{

  int current_num_sources = source_manager_.n_sources_;

  // update: frontend, feature_manager_, homography_manager_, source_manager_
  set_parameters(config);
  feature_manager_.set_parameters(config);
  homography_manager_.set_parameters(config);
  source_manager_.set_parameters(config);
  recognition_manager_.set_parameters(config);


  // Was there a change inthe number of sources?
  // if (current_num_sources != source_manager_.n_sources_)
  {
    // Clear the existing source information inside R-RANSAC
    params_.reset_sources();

    // Add each source with the corresponding new parameters
    for (auto&& src : source_manager_.measurement_sources_)
      params_.add_source(src->id_, src->has_velocity_, src->sigmaR_pos_, src->sigmaR_vel_);

    // Send updated parameters to R-RANSAC
    tracker_.set_parameters(params_);
  }

  ROS_INFO("visual frontend: parameters have been updated");
};

// ----------------------------------------------------------------------------

void VisualFrontend::callback_reconfigure_rransac(visual_mtt::rransacConfig& config, uint32_t level)
{

  // general
  params_.dt = config.dt;

  // motion model specific parameters
  params_.sigmaQ_vel = config.sigmaQ_vel;
  params_.alphaQ_vel = config.alphaQ_vel;
  params_.sigmaQ_jrk = config.sigmaQ_jrk;
  params_.alphaQ_jrk = config.alphaQ_jrk;

  // R-RANSAC specific parameters
  params_.Nw = config.Nw;
  params_.M = config.M;
  params_.tauR = config.tauR;
  params_.set_motion_model(static_cast<enum rransac::core::MotionModelType>(config.rransac_motion_model));

  // RANSAC specific parameters
  params_.ell = config.ell;
  params_.guided_sampling_threshold = config.guided_sampling_threshold;
  params_.tauR_RANSAC = config.tauR_RANSAC;
  params_.gamma = config.gamma;
  params_.set_motion_model_RANSAC(static_cast<enum rransac::core::MotionModelType>(config.ransac_motion_model));

  // model merging parameters
  params_.tau_vel_percent_diff = config.tau_vel_percent_diff;
  params_.tau_vel_abs_diff = config.tau_vel_abs_diff;
  params_.tau_angle_abs_diff = config.tau_angle_abs_diff;
  params_.tau_xpos_abs_diff = config.tau_xpos_abs_diff;
  params_.tau_ypos_abs_diff = config.tau_ypos_abs_diff;

  // model pruning parameters
  double percentage = config.surveillance_region;
  params_.field_max_x = std::abs(corner_[0].x*percentage);
  params_.field_max_y = std::abs(corner_[0].y*percentage);
  params_.tau_CMD_prune = config.tau_CMD_prune;

  // track (i.e., Good Model) parameters
  params_.tau_rho = config.tau_rho;
  params_.tau_CMD = config.tau_CMD;
  params_.tau_Vmax = config.tau_Vmax;
  params_.tau_T = config.tau_T;

  // Update the R-RANSAC Tracker with these new parameters
  tracker_.set_parameters(params_);

  ROS_INFO("rransac: parameters have been updated");
}

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

bool VisualFrontend::callback_srv_recognize_track()
{
//   // for debug and testing:
//   // std::cout << "callback pinged in the frontend" << std::endl;

//   res.id = recognition_manager_.identify_target(req.x, req.y);

//   return true;
}

// ----------------------------------------------------------------------------

void VisualFrontend::publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
{

  // Create the ROS message we will send
  visual_mtt::Tracks msg;

  for (int i=0; i<tracks.size(); i++)
  {
    visual_mtt::Track track;

    // General track information
    track.id            = tracks[i]->GMN;
    track.inlier_ratio  = tracks[i]->rho;

    // Position measurements
    track.position.x    = tracks[i]->xhat(0);
    track.position.y    = tracks[i]->xhat(1);

    // Velocity measurements
    track.velocity.x    = tracks[i]->xhat(2);
    track.velocity.y    = tracks[i]->xhat(3);

    // Error covariance: Convert col-major double to row-major float  ---  #MakeDonaldDrumpfAgain
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covfefe = tracks[i]->P.cast<float>();
    track.covariance.insert(track.covariance.end(), covfefe.data(), covfefe.data()+covfefe.size());

    // Add this track to the tracks msg
    msg.tracks.push_back(track);
  }

  // Include the original frame header and add the current time to the tracks
  // msg.header_frame = header_frame_;
  msg.header_update.stamp = ros::Time::now();
  msg.header_update.seq = ++iter_num_;

  // Attach utilization statistics to the message
  // msg.util = util_;

  // ROS publish
  pub_tracks.publish(msg);
}

// ----------------------------------------------------------------------------

void VisualFrontend::draw_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
{

  if (hd_frame_.empty())
    return;

  // std::cout << "Tracks: " << tracks.size() << std::endl;

  cv::Mat draw = hd_frame_.clone();

  static int max_num_tracks = 0;

  for (int i=0; i<tracks.size(); i++)
  {
    max_num_tracks = std::max(max_num_tracks, (int)tracks[i]->GMN);
    cv::Scalar color = colors_[tracks[i]->GMN];

    // Projecting Distances (tauR and velocity lines)
    // since the 2 important dimensions of the transform have roughly equal
    // eigenvalues, a raw distance in the normalized image plane can be
    // projected by scaling by that eigenvalue. camera_matrix_ is upper
    // triangular so the (0,0) element represents this scaling

    // get normalized image plane point
    cv::Point center;
    center.x = tracks[i]->xhat(0);
    center.y = tracks[i]->xhat(1);

    // treat points in the normalized image plane as a 3D points (homogeneous).
    // project the points onto the sensor (pixel space) for plotting.
    // use no rotation or translation (world frame = camera frame).
    std::vector<cv::Point3f> center_h; // homogeneous
    std::vector<cv::Point2f> center_d; // distorted

    // project the center point and the consensus set
    center_h.push_back(cv::Point3f(tracks[i]->xhat(0), tracks[i]->xhat(1), 1));
    for (int j=0; j<tracks[i]->CS.size(); j++)
      center_h.push_back(cv::Point3f(tracks[i]->CS[j]->pos(0), tracks[i]->CS[j]->pos(1), 1));

    cv::projectPoints(center_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), camera_matrix_, dist_coeff_, center_d);
    center = center_d[0];

    // draw circle with center at position estimate
    double radius = params_.tauR * camera_matrix_.at<double>(0,0);
    cv::circle(draw, center, (int)radius, color, 2, 8, 0);

    // draw red dot at the position estimate
    cv::circle(draw, center, 2, cv::Scalar(0, 0, 255), 2, 8, 0);

    // draw scaled velocity vector
    cv::Point velocity;
    double velocity_scale = 10; // for visibility
    velocity.x = tracks[i]->xhat(2) * camera_matrix_.at<double>(0,0) * velocity_scale;
    velocity.y = tracks[i]->xhat(3) * camera_matrix_.at<double>(0,0) * velocity_scale;
    cv::line(draw, center, center + velocity, color, 1, CV_AA);

    // draw model number and inlier ratio
    std::stringstream ssGMN;
    ssGMN << tracks[i]->GMN;
    int boldness = 2*((int)text_scale_);
    cv::putText(draw, ssGMN.str().c_str(), cv::Point(center.x + 5, center.y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.85*text_scale_, cv::Scalar(0, 0, 210), boldness);

    // draw consensus sets
    for (int j=1; j<center_d.size(); j++)
    {
      center = center_d[j];
      cv::circle(draw, center, 2, color, -1, 8, 0);
    }
  }

  // draw top-left box
  char text[40];
  cv::Point bl_corner = cv::Point(165, 18)*text_scale_;
  cv::Point corner_offset = cv::Point(0, 20)*text_scale_;
  cv::Point text_offset = cv::Point(5, 13)*text_scale_;
  double text_size = 0.5*text_scale_;

  sprintf(text, "Frame: %d", frame_);
  cv::Point corner = cv::Point(5,5)*text_scale_;
  cv::rectangle(draw, corner, corner + bl_corner, cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + text_offset, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));

  sprintf(text, "Total models: %d", max_num_tracks);
  corner += corner_offset;
  cv::rectangle(draw, corner, corner + bl_corner, cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + text_offset, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));

  sprintf(text, "Current models: %d", (int)tracks.size());
  corner += corner_offset;
  cv::rectangle(draw, corner, corner + bl_corner, cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + text_offset, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));

  // utilization rectangle background color
  // cv::Scalar background = (util_.total > 0.9) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255);
  cv::Scalar background = cv::Scalar(255, 255, 255);

  // sprintf(text, "Utilization: %d%%", (int)(util_.total*100));
  sprintf(text, "Utilization: %d%%", 0);
  corner += corner_offset;
  cv::rectangle(draw, corner, corner + bl_corner, background, -1);
  cv::putText(draw, text, corner + text_offset, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));

  // Resize the image according to the scale
  cv::Mat resized;
  cv::Size size(draw.cols*pub_scale_, draw.rows*pub_scale_);
#if OPENCV_CUDA
  cv::cuda::GpuMat draw_cuda, resized_cuda;
  draw_cuda.upload(draw);
  cv::cuda::resize(draw_cuda, resized_cuda, size, 0, 0, cv::INTER_AREA);
  resized_cuda.download(resized);
#else
  cv::resize(draw, resized, size, 0, 0, cv::INTER_AREA);
#endif

  // Publish over ROS network
  cv_bridge::CvImage image_msg;
  image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_msg.image = resized;
  // image_msg.header = header_frame_;
  pub_tracks_video.publish(image_msg.toImageMsg());
}

}