#include "visual_frontend/visual_frontend.h"

namespace visual_frontend {

VisualFrontend::VisualFrontend()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh_private("~");


  // Load static parameters.
  std::string filename;
  nh_private.param<std::string>("static_param_filename",filename, "");
  static_params_.Initialize(filename);

  // ROS Messages Throttle
  static_params_.GetParam("visual_frontend/message_output_period", sys_.message_output_period_, 60);
  ROS_INFO_STREAM("ROS messages will be printed to the scrren with a minimum period of " << sys_.message_output_period_ << " seconds");


  /////////////////////////////////////////////////////////////////////////
  // Initialize the managers and plugins
  std::vector<std::string> measurement_manager_plugin_whitelist, 
                           transform_manager_plugin_whitelist,
                           feature_manager_plugin_whitelist;

  // get parameters from param server that are not dynamically reconfigurable
  nh_private.param<bool>("tuning", sys_.tuning_, 0);
  nh_private.getParam("measurement_manager/plugins",measurement_manager_plugin_whitelist);
  nh_private.getParam("transform_manager/plugins",transform_manager_plugin_whitelist);
  nh_private.getParam("feature_manager/plugins",feature_manager_plugin_whitelist);

  // Load plugins
  measurement_manager_.LoadPlugins(measurement_manager_plugin_whitelist, static_params_);
  transform_manager_.LoadPlugins(transform_manager_plugin_whitelist, static_params_);
  feature_manager_.LoadPlugins(feature_manager_plugin_whitelist, static_params_);

  if (sys_.tuning_)
    ROS_WARN("tuning mode enabled");

  /////////////////////////////////////////////////////////////////////////
  // Setup ROS topics and dynamic reconfigures

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_video  = it.subscribeCamera("video", 10, &VisualFrontend::CallbackVideo,  this);
  pub_tracks       = nh_.advertise<visual_mtt::Tracks>("tracks", 1);
  pub_tracks_video = it.advertiseCamera("tracks_video/image_raw", 1);
  pub_transform_   = nh_.advertise<std_msgs::Float32MultiArray>("transform", 1);

  // establish dynamic reconfigure and load defaults (callback runs once here)
  auto func1 = std::bind(&VisualFrontend::CallbackReconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func1);

  // establish dynamic reconfigure and load defaults (callback runs once here)
  ros::NodeHandle nh_private_rransac("rransac");
  rransac_server_.reset(new dynamic_reconfigure::Server<visual_mtt::rransacConfig>(nh_private_rransac));
  auto func2 = std::bind(&VisualFrontend::CallbackReconfigureRransac, this, std::placeholders::_1, std::placeholders::_2);
  rransac_server_->setCallback(func2);

  // populate plotting colors
  colors_ = std::vector<cv::Scalar>();
  for (int i = 0; i < 1000; i++)
    colors_.push_back(cv::Scalar(std::rand() % 256, std::rand() % 256, std::rand() % 256));


  // establish librransac good model elevation event callback
  params_.set_elevation_callback(std::bind(&VisualFrontend::CallbackElevationEvent, this, std::placeholders::_1, std::placeholders::_2));
}

// ----------------------------------------------------------------------------

void VisualFrontend::CallbackVideo(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo)
{

  auto tic = ros::Time::now();

  // save the camera parameters and frame timestamp
  static std_msgs::Header header_frame_last;
  header_frame_last = header_frame_;
  header_frame_ = data->header;

  //
  // Estimate FPS
  //

  // LPF alpha: Converge quickly at first
  double alpha = (frame_ < 30) ? 0.95 : alpha_;

  // enforce realistic time differences (for rosbag looping)
  ros::Duration elapsed = header_frame_.stamp - header_frame_last.stamp;
  if (!(elapsed.toSec()<=0 || elapsed.toSec()>1))
    fps_ = alpha*(1/elapsed.toSec()) + (1-alpha)*fps_;



  // Only process every Nth frame
  if (frame_++ % frame_stride_ != 0)
    return;


  // convert message data into OpenCV type cv::Mat
  sys_.SetHDFrame(cv_bridge::toCvCopy(data, "bgr8")->image);

  // save camera parameters one time
  if (!sys_.cam_info_received_)
  {
    // camera_matrix_ (K) is 3x3
    // dist_coeff_    (D) is a column vector of 4, 5, or 8 elements
    cv::Mat camera_matrix(3, 3, CV_64FC1);
    cv::Mat dist_coeff(cinfo->D.size(), 1, CV_64FC1);

    // convert rosmsg vectors to cv::Mat
    for(int i=0; i<9; i++)
      camera_matrix.at<double>(i/3, i%3) = cinfo->K[i];

    for(int i=0; i<cinfo->D.size(); i++)
      dist_coeff.at<double>(i, 0) = cinfo->D[i];

    sys_.SetCameraParams(camera_matrix, dist_coeff);
    sys_.SetScaledCameraParams();

    // update surveillance region based on normalized image plane
    // corner_.push_back(cv::Point2f(0,0));
    cv::undistortPoints(corner_, corner_, sys_.hd_camera_matrix_, sys_.dist_coeff_);

    double percentage;
    nh_.param<double>("rransac/surveillance_region", percentage, 0);

    params_.field_max_x = std::abs(corner_[0].x*percentage);
    params_.field_max_y = std::abs(corner_[0].y*percentage);
    tracker_.set_parameters(params_);

  }

  //
  // Initial image processing
  //

  sys_.SetSDFrame();  
  sys_.ClearFlags();

  // update the target recognition algorithm with the recent frame
  recognition_manager_.update_image(sys_.hd_frame_);

  // use track information (from last iteration) to update target descriptors
  if (sys_.tracks_.size() > 0)
     recognition_manager_.update_descriptors(sys_.tracks_);

  t_other_ += (ros::Time::now() - tic).toSec();

  /////////////////////////////////////////////////////
  // Feature Manager: LKT Tracker, ORB-BN, etc
  //

  tic = ros::Time::now();
  feature_manager_.FindCorrespondences(sys_);
  double t_features = (ros::Time::now() - tic).toSec();

  ////////////////////////////////////////////////////
  // Transform Manager
  //

  // calculate the transform
  tic = ros::Time::now();
  transform_manager_.CalculateTransform(sys_);
  double t_homography = (ros::Time::now() - tic).toSec();

  /////////////////////////////////////////////////////
  // Measurement Generation from multiple sources
  //

  tic = ros::Time::now();
  measurement_manager_.GenerateMeasurements(sys_);
  double t_measurements = (ros::Time::now() - tic).toSec();

  //
  // R-RANSAC Tracker
  //

  tic = ros::Time::now();
  UpdateRRANSAC();
  double t_rransac = (ros::Time::now() - tic).toSec();

  //
  // Calculate utiliization
  //
  tic = ros::Time::now();
  util_.number_of_rransac_measurements = sys_.num_of_measurements_;

  // Save stride
  double t_available = (1/fps_)*frame_stride_;

  // LPF alpha: Converge quickly at first
  alpha = (frame_ < 30) ? 0.95 : 1/(time_constant_/t_available + 1);

  util_.time_available          = t_available;
  util_.feature_manager         = alpha*(t_features/t_available*100)      + (1-alpha)*util_.feature_manager;
  util_.homography_manager      = alpha*(t_homography/t_available*100)    + (1-alpha)*util_.homography_manager;
  util_.measurement_generation  = alpha*(t_measurements/t_available*100)  + (1-alpha)*util_.measurement_generation;
  util_.other                   = alpha*(t_other_/t_available*100)        + (1-alpha)*util_.other;
  util_.rransac                 = alpha*(t_rransac/t_available*100)       + (1-alpha)*util_.rransac;

  double total = util_.feature_manager + util_.homography_manager + util_.measurement_generation + util_.rransac;
  util_.total = alpha*total + (1-alpha)*util_.total;

  //
  // Publish results
  //

  // publish the tracks and homography matrix onto ROS network
  PublishTracks(sys_.tracks_);
  PublishTransform();

  // generate visualization, but only if someone is listening
  if (pub_tracks_video.getNumSubscribers() > 0 && (pub_frame_++ % publish_frame_stride_ == 0)) {
    const cv::Mat drawing = DrawTracks(sys_.tracks_);

    // Publish over ROS network
    if (!drawing.empty()) {
      cv_bridge::CvImage image_msg;
      image_msg.encoding = sensor_msgs::image_encodings::BGR8;
      image_msg.image = drawing;
      image_msg.header = header_frame_;
      pub_tracks_video.publish(image_msg.toImageMsg(), cinfo);
    }
  }

  t_other_ = (ros::Time::now() - tic).toSec();
}

// ----------------------------------------------------------------------------

void VisualFrontend::CallbackReconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level)
{

  // int current_num_sources = measurement_manager_.n_sources_;

  // update: frontend, feature_manager_, homography_manager_, measurement_manager_
  SetParameters(config);
  feature_manager_.SetParameters(config);
  transform_manager_.SetParameters(config);
  measurement_manager_.SetParameters(config);
  recognition_manager_.set_parameters(config);

  /* TODO
  it should not be reseting every time. 
  */


  // Was there a change inthe number of sources?
  // if (current_num_sources != measurement_manager_.n_sources_)
  {
    // Clear the existing source information inside R-RANSAC
    params_.reset_sources();

    /* TODO
    Make sources private, and get information through a method call.
    */
    // Add each source with the corresponding new parameters
    for (auto&& src : measurement_manager_.measurement_sources_)
    {
      params_.add_source(src->id_, src->has_velocity_, src->sigmaR_pos_, src->sigmaR_vel_);
    }

    // Send updated parameters to R-RANSAC
    tracker_.set_parameters(params_);

  }

  ROS_INFO("visual frontend: parameters have been updated");
};

// ----------------------------------------------------------------------------

void VisualFrontend::CallbackReconfigureRransac(visual_mtt::rransacConfig& config, uint32_t level)
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

void VisualFrontend::SetParameters(visual_mtt::visual_frontendConfig& config)
{
  frame_stride_ = config.frame_stride;
  publish_frame_stride_ = config.published_frame_stride;
  pub_scale_ = config.published_video_scale;

  sys_.SetResizeScale(config.resize_scale);
  sys_.SetScaledCameraParams();
}

// ----------------------------------------------------------------------------

uint32_t VisualFrontend::CallbackElevationEvent(double x, double y) {
  uint32_t id = recognition_manager_.identify_target(x, y);
  return id;
}

// ----------------------------------------------------------------------------

void VisualFrontend::PublishTracks(const std::vector<rransac::core::ModelPtr>& tracks)
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
  msg.header_frame = header_frame_;
  msg.header_update.stamp = ros::Time::now();
  msg.header_update.seq = ++iter_num_;

  // Attach utilization statistics to the message
  msg.util = util_;

  // ROS publish
  pub_tracks.publish(msg);
}

// ----------------------------------------------------------------------------

void VisualFrontend::PublishTransform()
{

    cv::Mat T = sys_.transform_;
    std_msgs::Float32MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].label = "height";
    msg.layout.dim[0].size = T.rows;
    msg.layout.dim[0].stride = T.rows*T.cols;
    msg.layout.dim[1].label = "width";
    msg.layout.dim[1].size = T.cols;
    msg.layout.dim[1].stride = T.cols;
    msg.layout.data_offset = 0;
    std::vector<float> vec;
    for (int i = 0; i < T.rows; i++)
    {
        vec.insert(vec.end(), T.ptr<float>(i), T.ptr<float>(i)+T.cols);
    }
    msg.data = vec;
    pub_transform_.publish(msg);
}

// ----------------------------------------------------------------------------

cv::Mat VisualFrontend::DrawTracks(const std::vector<rransac::core::ModelPtr>& tracks)
{

  if (sys_.hd_frame_.empty())
    return sys_.hd_frame_;

  cv::Mat draw = sys_.hd_frame_.clone();

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

    cv::projectPoints(center_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sys_.hd_camera_matrix_, sys_.dist_coeff_, center_d);
    center = center_d[0];

    // draw circle with center at position estimate
    double radius = params_.tauR * sys_.hd_camera_matrix_.at<double>(0,0);
    cv::circle(draw, center, (int)radius, color, 2, 8, 0);

    // draw red dot at the position estimate
    cv::circle(draw, center, 2, cv::Scalar(0, 0, 255), 2, 8, 0);

    // draw scaled velocity vector
    cv::Point velocity;
    double velocity_scale = 10; // for visibility
    velocity.x = tracks[i]->xhat(2) * sys_.hd_camera_matrix_.at<double>(0,0) * velocity_scale;
    velocity.y = tracks[i]->xhat(3) * sys_.hd_camera_matrix_.at<double>(0,0) * velocity_scale;
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
  cv::Scalar background = (util_.total > 90) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255);

  sprintf(text, "Utilization: %d%%", (int)(util_.total));
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

  return resized;
}

// ----------------------------------------------------------------------------

void VisualFrontend::UpdateRRANSAC()
{
  // Feed rransac measurements
  for (auto& meas_src : sys_.measurements_)
  {
    if(meas_src.has_velocity)
      tracker_.add_measurements<CVPoint2fAccess>(meas_src.meas_pos, meas_src.meas_vel, meas_src.id);
    else
      tracker_.add_measurements<CVPoint2fAccess>(meas_src.meas_pos, meas_src.id);
  }

  // Apply transform
  Eigen::Matrix3f TT;
  cv::cv2eigen(sys_.transform_, TT);
  Eigen::Projective2d T(TT.cast<double>());
  tracker_.apply_transformation(T);

  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  sys_.tracks_ = tracker_.run();
}

}
