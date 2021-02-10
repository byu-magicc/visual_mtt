#include "visual_frontend/visual_frontend.h"

namespace visual_frontend {

VisualFrontend::VisualFrontend()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh_private("~");


  // Load static parameters.
  std::string static_param_filename,gnsac_solver_filename;
  nh_private.param<std::string>("static_param_filename",static_param_filename, "");
  nh_private.param<std::string>("gnsac_solver_filename",gnsac_solver_filename,"");
  static_params_.Initialize(static_param_filename);
  feature_manager_.Initialize(gnsac_solver_filename);

  // ROS Messages Throttle
  static_params_.GetParam("visual_frontend/message_output_period", sys_.message_output_period_, 60);
  ROS_INFO_STREAM("ROS messages will be printed to the scrren with a minimum period of " << sys_.message_output_period_ << " seconds");


  /////////////////////////////////////////////////////////////////////////
  // Initialize the managers and plugins
  std::vector<std::string> measurement_manager_plugin_whitelist, 
                           transform_manager_plugin_whitelist,
                           feature_manager_plugin_whitelist;

  // get parameters from param server that are not dynamically reconfigurable
  std::string picture_file_path;
  nh_private.param<bool>("tuning", sys_.tuning_, 0);
  nh_private.param<std::string>("picture_file_path", picture_file_path, "");
  sys_.SetPictureFilepath(picture_file_path);

  nh_private.getParam("measurement_manager/plugins",measurement_manager_plugin_whitelist);
  nh_private.getParam("transform_manager/plugins",transform_manager_plugin_whitelist);
  nh_private.getParam("feature_manager/plugins",feature_manager_plugin_whitelist);

  // Load plugins
  measurement_manager_.LoadPlugins(measurement_manager_plugin_whitelist, static_params_);
  transform_manager_.LoadPlugins(transform_manager_plugin_whitelist, static_params_);
  feature_manager_.LoadPlugins(feature_manager_plugin_whitelist, static_params_);

  // Update required plugin frames
  sys_.RegisterPluginFrames(sys_.default_frames_required_);
  sys_.RegisterPluginFrames(measurement_manager_.frames_required_);
  sys_.RegisterPluginFrames(transform_manager_.frames_required_);
  sys_.RegisterPluginFrames(feature_manager_.frames_required_);
#if OPENCV_CUDA
  sys_.RegisterPluginCUDAFrames(sys_.default_cuda_frames_required_);
  sys_.RegisterPluginCUDAFrames(measurement_manager_.cuda_frames_required_);
  sys_.RegisterPluginCUDAFrames(transform_manager_.cuda_frames_required_);
  sys_.RegisterPluginCUDAFrames(feature_manager_.cuda_frames_required_);
#endif

  if (sys_.tuning_)
    ROS_WARN("tuning mode enabled");
    sys_.RegisterFrame(common::SD);

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


  ////////////////////////////////////////////////////////////////////////////////////
  // Setup sources for RRANSAC
  for (auto&& sources : measurement_manager_.measurement_sources_) {
    rransac_.AddSource(sources->source_parameters_);
  }

  rransac_sys_ = rransac_.GetSystemInformation();


  ////////////////////////////////////////////////////////////////////////
  // Image drawing and saving parameters
  name_ = "Visual MTT";
  pic_params_.pic_num = 0;
  pic_params_.file_name = "Tracks";
  drawn_ = false;

}

// ----------------------------------------------------------------------------

void VisualFrontend::CallbackVideo(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo)
{

  auto tic = ros::WallTime::now();

  // save the camera parameters and frame timestamp
  static std_msgs::Header header_frame_last;
  header_frame_last = header_frame_;
  header_frame_ = data->header;

  // set time (this should come from the header file, but it isn't gauranteed that the header file has time information. )
  sys_.current_time_ = header_frame_.stamp.toSec();

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
  sys_.SetHDFrame(cv_bridge::toCvShare(data, "bgr8")->image);

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



  }



  //
  // Initial image processing
  //

#if OPENCV_CUDA
  sys_.SetCUDAFrames();
#endif
  sys_.SetFrames();
  sys_.ClearFlags();

  // // update the target recognition algorithm with the recent frame
  // recognition_manager_.update_image(sys_.hd_frame_);

  // // use track information (from last iteration) to update target descriptors
  // if (sys_.tracks_.size() > 0)
  //    recognition_manager_.update_descriptors(sys_.tracks_);

  t_other_ += (ros::WallTime::now() - tic).toSec();

  /////////////////////////////////////////////////////
  // Feature Manager: LKT Tracker, ORB-BN, etc
  //

  tic = ros::WallTime::now();
  feature_manager_.FindCorrespondences(sys_);
  double t_features = (ros::WallTime::now() - tic).toSec();

  ////////////////////////////////////////////////////
  // Transform Manager
  //

  // calculate the transform
  tic = ros::WallTime::now();
  transform_manager_.CalculateTransform(sys_);
  double t_homography = (ros::WallTime::now() - tic).toSec();

  /////////////////////////////////////////////////////
  // Measurement Generation from multiple sources
  //

  tic = ros::WallTime::now();
  measurement_manager_.GenerateMeasurements(sys_);
  double t_measurements = (ros::WallTime::now() - tic).toSec();

  //
  // R-RANSAC Tracker
  //

  tic = ros::WallTime::now();
  UpdateRRANSAC();
  double t_rransac = (ros::WallTime::now() - tic).toSec();

  //
  // Calculate utiliization
  //
  tic = ros::WallTime::now();
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

  double total = util_.feature_manager + util_.homography_manager + util_.measurement_generation + util_.rransac + util_.other ;
  util_.total = alpha*total + (1-alpha)*util_.total;

  //
  // Publish results
  //

  // publish the tracks and homography matrix onto ROS network
  PublishTracks(rransac_sys_->good_models_);
  PublishTransform();

  // generate visualization, but only if someone is listening or in tuning mode
  bool listening_or_tuning_mode = (pub_tracks_video.getNumSubscribers() > 0) || sys_.tuning_;
  bool pub_frame_ready = (pub_frame_++ % publish_frame_stride_ == 0);
  if (listening_or_tuning_mode > 0 && pub_frame_ready) {
    const cv::Mat drawing = DrawTracks();

    if(sys_.tuning_ && drawn_ == false)
    {
      cv::namedWindow(name_,CV_WINDOW_NORMAL);
      cv::setMouseCallback(name_,sys_.TakePicture, &pic_params_);
      cv::resizeWindow(name_,sys_.sd_res_.width,sys_.sd_res_.height);
      
    }

    if(!drawing.empty() && sys_.tuning_)
    {
      drawn_ = true;
      cv::imshow(name_, drawing);
      pic_params_.img = drawing.clone();
    }

    // Publish over ROS network
    if (!drawing.empty()) {
      cv_bridge::CvImage image_msg;
      image_msg.encoding = sensor_msgs::image_encodings::BGR8;
      image_msg.image = drawing;
      image_msg.header = header_frame_;
      pub_tracks_video.publish(image_msg.toImageMsg(), cinfo);
    }
  }

  t_other_ = (ros::WallTime::now() - tic).toSec();

  sys_.ResetFrames();
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





  // rransac needs to update the source parameters if they changed
  for (auto&& src : measurement_manager_.measurement_sources_)
  {
    if (src->source_parameters_changed_) {
      rransac_.ChangeSourceParameters(src->source_parameters_);
      src->source_parameters_changed_ = false;
  }
  }

  ROS_INFO("visual frontend: parameters have been updated");
};

// ----------------------------------------------------------------------------

void VisualFrontend::CallbackReconfigureRransac(visual_mtt::rransacConfig& config, uint32_t level)
{

  // general
  rransac_params_.meas_time_window_ = config.meas_time_window;
  rransac_params_.transform_consensus_set_ = config.transform_consensus_set;

  // Cluster Parameters
  rransac_params_.cluster_time_threshold_ = config.cluster_time_threshold;
  rransac_params_.cluster_velocity_threshold_ = config.cluster_velocity_threshold;
  rransac_params_.cluster_position_threshold_ = config.cluster_position_threshold;
  rransac_params_.cluster_min_size_requirement_ = config.cluster_min_size_requirement;

  // RANSAC Parameters
  rransac_params_.RANSAC_max_iters_ = config.RANSAC_max_iters;
  rransac_params_.RANSAC_score_stopping_criteria_ = config.RANSAC_score_stopping_criteria;
  rransac_params_.RANSAC_score_minimum_requirement_ = config.RANSAC_score_minimum_requirement;
  rransac_params_.RANSAC_minimum_subset_ = config.RANSAC_minimum_subset;

   // Model Manager Parameters
  rransac_params_.track_good_model_threshold_ = config.track_good_model_threshold;
  rransac_params_.track_max_missed_detection_time_ = config.track_max_missed_detection_time;
  rransac_params_.track_similar_tracks_threshold_ = config.track_similar_tracks_threshold;
  rransac_params_.track_max_num_tracks_ = config.track_max_num_tracks;

  // Nonlinear LMLE
  rransac_params_.nonlinear_innov_cov_id_ = config.nonlinear_innov_cov_id;

#if TRACKING_SE2
  rransac_params_.process_noise_covariance_ = Eigen::Matrix<double,5,5>::Identity();
  rransac_params_.process_noise_covariance_.diagonal() << config.covQ_pos, config.covQ_pos, config.covQ_pos, config.covQ_vel, covQ_vel;
#else
  rransac_params_.process_noise_covariance_ = Eigen::Matrix<double,4,4>::Identity();
  rransac_params_.process_noise_covariance_.diagonal() << config.covQ_pos, config.covQ_pos, config.covQ_vel, config.covQ_vel;
#endif

  // Update the R-RANSAC Tracker with these new parameters
  rransac_.SetSystemParameters(rransac_params_);

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

void VisualFrontend::PublishTracks(const std::vector<RR_Model*> tracks)
{

  // Create the ROS message we will send
  visual_mtt::Tracks msg;

  for (int i=0; i<tracks.size(); i++)
  {
    visual_mtt::Track track;

    // General track information
    track.id                 = tracks[i]->label_;
    track.model_likelihood  = tracks[i]->model_likelihood_;

#if TRACKING_SE2
    // Position measurements
    track.position.x    = tracks[i]->state_.g_.t_(0);
    track.position.y    = tracks[i]->state_.g_.t_(1);
    track.R = std::vector<float>(tracks[i]->state_.g_.R_(0,0).tracks[i]->state_.g_.R_(0,1),tracks[i]->state_.g_.R_(1,0),tracks[i]->state_.g_.R_(1,1)   );

    // Velocity measurements
    track.velocity.x    = tracks[i]->state_.u_.p_(0);
    track.velocity.y    = tracks[i]->state_.u_.p_(1);
    track.omega = tracks[i]->state_.u_.th_(0);
#else // R2
    // Position measurements
    track.position.x    = tracks[i]->state_.g_.data_(0);
    track.position.y    = tracks[i]->state_.g_.data_(1);

    // Velocity measurements
    track.velocity.x    = tracks[i]->state_.u_.data_(0);
    track.velocity.y    = tracks[i]->state_.u_.data_(1);
#endif


    // Error covariance: Convert col-major double to row-major float  ---  #MakeDonaldDrumpfAgain
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covfefe = tracks[i]->err_cov_.cast<float>();
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

cv::Mat VisualFrontend::DrawTracks()
{

  if (sys_.GetFrame(common::HD).empty())
    return sys_.GetFrame(common::HD);

  cv::Mat draw = sys_.GetFrame(common::HD).clone();

  static int max_num_tracks = rransac_sys_->model_label_;
  double x_pos = 0;
  double y_pos = 0;
  double x_vel = 0;
  double y_vel = 0;
  Eigen::Matrix<double,2,1> vel;

  for (auto&& track : rransac_sys_->good_models_)
  {
    
    cv::Scalar color = colors_[track->label_];
    x_pos = track->state_.g_.data_(0,0);
    y_pos = track->state_.g_.data_(1,0);

#if TRACKING_SE2
  vel = track->state_.g_.R_ * track->state_.u_.p_;
#else // R2
  vel = track->state_.u_.data_;
#endif

  x_vel = vel(0,0);
  y_vel = vel(1,0);

    // Projecting Distances (tauR and velocity lines)
    // since the 2 important dimensions of the transform have roughly equal
    // eigenvalues, a raw distance in the normalized image plane can be
    // projected by scaling by that eigenvalue. camera_matrix_ is upper
    // triangular so the (0,0) element represents this scaling

    // get normalized image plane point
    cv::Point center;
    center.x = x_pos;
    center.y = y_pos;

    // treat points in the normalized image plane as a 3D points (homogeneous).
    // project the points onto the sensor (pixel space) for plotting.
    // use no rotation or translation (world frame = camera frame).
    std::vector<cv::Point3f> center_h; // homogeneous
    std::vector<cv::Point2f> center_d; // distorted

    // project the center point and the consensus set
    center_h.push_back(cv::Point3f(x_pos, y_pos, 1));
    // for (const auto& outer_iter = track->cs_.consensus_set_.begin(); outer_iter != track->cs_.consensus_set_.end(); ++ outer_iter) {
    //   for (const auto& inner_iter = outer_iter->begin(); inner_iter != outer_iter->end(); ++inner_iter) {
    //     center_h.push_back(cv::Point3f(inner_iter->pose(0,0), inner_iter->pose(1,0), 1));
    //   }
    // }

    cv::projectPoints(center_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sys_.hd_camera_matrix_, sys_.dist_coeff_, center_d);
    center = center_d[0];

    // TODO:: Add this back in
    // // draw circle with center at position estimate
    // double radius = params_.tauR * sys_.hd_camera_matrix_.at<double>(0,0);
    // cv::circle(draw, center, (int)radius, color, 2, 8, 0);


    ////////////////////////////////////////////////////////////////////
    // Draw validation region
    ///////////////////////////////////////////////////////////////////
        auto& source = rransac_sys_->sources_.front();
        unsigned int source_index = source.params_.source_index_;
        Eigen::MatrixXd S = track->GetInnovationCovariance(rransac_sys_->sources_,source_index);
        Eigen::Matrix2d S_pos = S.block(0,0,2,2);
        Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver;
        eigen_solver.compute(S_pos);
        Eigen::Vector2cd eigen_values = eigen_solver.eigenvalues();
        Eigen::Matrix2cd eigen_vectors = eigen_solver.eigenvectors();
        double th = 0;
        // Make sure that the x component is positive
        if (std::real(eigen_vectors(0,0)) < 0) {
            eigen_vectors.block(0,0,2,1)*=-1;
        }
        if (std::real(eigen_vectors(0,1)) < 0) {
            eigen_vectors.block(0,1,2,1)*=-1;
        }


        double scale = std::sqrt(source.params_.gate_threshold_)*sys_.hd_camera_matrix_.at<double>(0,0);
        if (std::real(eigen_vectors(0,0)*eigen_vectors(1,0)) < 0) {
            th = std::real(eigen_vectors(0,0))*180/M_PI;
            cv::Size size(std::sqrt(std::norm(eigen_values(0)))*scale,std::sqrt(std::norm(eigen_values(1)))*scale);
            cv::ellipse(draw,center,size,th, 0,360,color,2, cv::LINE_AA);
        } else {
            th = std::real(eigen_vectors(0,1))*180/M_PI;
            cv::Size size(std::sqrt(std::norm(eigen_values(1)))*scale,std::sqrt(std::norm(eigen_values(0)))*scale);
            cv::ellipse(draw,center,size,th, 0,360,color,2, cv::LINE_AA);
        }
    



    // draw red dot at the position estimate
    cv::circle(draw, center, 2, cv::Scalar(0, 0, 255), 2, 8, 0);

    // draw scaled velocity vector
    cv::Point velocity;
    double velocity_scale = 1; // for visibility
    velocity.x = x_vel * sys_.hd_camera_matrix_.at<double>(0,0) * velocity_scale;
    velocity.y = y_vel * sys_.hd_camera_matrix_.at<double>(0,0) * velocity_scale;
    cv::line(draw, center, center + velocity, color, 1, CV_AA);

    // draw model number and inlier ratio
    std::stringstream ssGMN;
    ssGMN << track->label_;
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

  sprintf(text, "Current models: %d", (int)rransac_sys_->good_models_.size());
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


  // Apply transform
  Eigen::Matrix3d TT;
  cv::cv2eigen(sys_.transform_, TT);

  std::cout << "tt: " << std::endl << TT << std::endl;
  // std::cout << "transform: " << std::endl << sys_.transform_ << std::endl;

  for (auto& m : sys_.measurements_) {
    std::cout << "p: " << std::endl << m.pose << std::endl;
  }

  rransac_.AddMeasurements(sys_.measurements_,TT);

  std::cout << "tt r: " << std::endl << rransac_sys_->transformaion_.GetData() << std::endl;

  rransac_.RunTrackInitialization();
  rransac_.RunTrackManagement();

}

}
