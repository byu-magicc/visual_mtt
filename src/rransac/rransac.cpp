#include "rransac/rransac.h"

namespace rransac {

RRANSAC::RRANSAC()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh_private("~");

  // host the param update ROS service server
  srv_params_ = nh_private.advertiseService("set_params", &RRANSAC::callback_srv_params, this);

  // connect the track recognition ROS service client
  srv_recognize_track_ = nh_.serviceClient<visual_mtt::RecognizeTrack>("visual_frontend/recognize_track");
  srv_recognize_track_.waitForExistence(ros::Duration(10.0));
  ROS_INFO("rransac: services successfully established, starting node");

  // establish librransac good model elevation event callback
  params_.set_elevation_callback(std::bind(&RRANSAC::callback_elevation_event, this, std::placeholders::_1, std::placeholders::_2));

  // instantiate the rransac::Tracker library class
  tracker_ = rransac::Tracker(params_);

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_video        = it.subscribeCamera("video", 10, &RRANSAC::callback_video, this);
  sub_scan         = nh_.subscribe("measurements", 1, &RRANSAC::callback_scan, this);
  sub_stats        = nh_.subscribe("stats", 1, &RRANSAC::callback_stats, this);
  pub_tracks       = nh_.advertise<visual_mtt::Tracks>("tracks", 1);
  pub_tracks_video = it.advertise("tracks_video", 1);

  // initialize the top left corner of normalized image plane with zeros
  corner_.push_back(cv::Point2f(0,0));

  // populate plotting colors
  colors_ = std::vector<cv::Scalar>();
  for (int i = 0; i < 1000; i++)
    colors_.push_back(cv::Scalar(std::rand() % 256, std::rand() % 256, std::rand() % 256));

  // establish dynamic reconfigure and load defaults (callback runs once here)
  auto func = std::bind(&RRANSAC::callback_reconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

uint32_t RRANSAC::callback_elevation_event(double x, double y) {
  // Using the rransac::core::Parameters object, this function is passed to
  // R-RANSAC to act as the model elevation event callback. Inside this
  // function a ROS service call is made to the frontend, providing the
  // normalized image coordinates of the elevated model. The frontend returns
  // an old ID of the recognized historical track. The return here passes
  // this result directly into R-RANSAC.

  visual_mtt::RecognizeTrack srv;
  srv.request.x = x;
  srv.request.y = y;

  // All ROS service calls are blocking by nature (and have no timeout). This
  // means that srv_params_ may have a call queued when the following service
  // call is made. Such would put both nodes waiting on each other. In order to
  // prevent such gridlock, clear the service call queue. This returns a fail
  // signal to the caller, an event which is accommodated on that end.

  // clear the param update server call queue to prevent gridlock
  srv_params_.shutdown();
  uint32_t idx;

  if (srv_recognize_track_.call(srv))
  {
    // service call was successful, return the provided ID to R-RANSAC
    idx = srv.response.id;
  }
  else
  {
    // service call was unsuccessful
    ROS_ERROR("failed to call recognize track service.");

    // return no ID to R-RANSAC
    idx = 0;
  }

  // restart the param update server
  ros::NodeHandle nh_private("~");
  srv_params_ = nh_private.advertiseService("set_params", &RRANSAC::callback_srv_params, this);

  return idx;
}


void RRANSAC::callback_reconfigure(visual_mtt::rransacConfig& config, uint32_t level)
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

bool RRANSAC::callback_srv_params(visual_mtt::RRANSACParams::Request &req, visual_mtt::RRANSACParams::Response &res)
{
  pub_scale_  = req.published_video_scale;
  text_scale_ = req.text_scale;

  // See if any source parameters have changed
  bool changed = false;
  if (req.n_sources!=req_last_.n_sources) changed = true;
  else
  {
    // loop through sources before and after
    for (int i=0; i<req.n_sources; i++)
    {
      if (req.id[i]!=req_last_.id[i])                 changed = true;
      if (req.sigmaR_pos[i]!=req_last_.sigmaR_pos[i]) changed = true;
      if (req.sigmaR_vel[i]!=req_last_.sigmaR_vel[i]) changed = true;
    }
  }

  if (changed)
  {
    // Clear the existing source information inside R-RANSAC
    params_.reset_sources();

    // Add each source with the corresponding new parameters
    int id;
    bool has_velocity;
    double sigmaR_pos;
    double sigmaR_vel;
    for (int i=0; i<req.n_sources; i++)
    {
      id = req.id[i];
      has_velocity = req.has_velocity[i];
      sigmaR_pos = req.sigmaR_pos[i];
      sigmaR_vel = req.sigmaR_vel[i];
      params_.add_source(id, has_velocity, sigmaR_pos, sigmaR_vel);
    }

    // Remember the request (new source parameters)
    req_last_ = req;

    // Send updated parameters to R-RANSAC
    tracker_.set_parameters(params_);
  }

  ROS_INFO("rransac: service call param update");
}

// ----------------------------------------------------------------------------

void RRANSAC::callback_scan(const visual_mtt::RRANSACScanPtr& rransac_scan)
{
  auto tic = ros::Time::now();

  // Access the homography from the ROS message, convert to Projective2d, and give to R-RANSAC
  Eigen::Matrix3f H = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(rransac_scan->homography.data());
  Eigen::Projective2d T(H.cast<double>());
  tracker_.apply_transformation(T);


  // Add each source's measurements along with its ID to the R-RANSAC Tracker
  util_.number_of_rransac_measurements = 0;
  for (auto src = rransac_scan->sources.begin(); src != rransac_scan->sources.end(); src++)
  {
    if (src->dimensionality == 2)
    {
      if (src->has_velocity)
        tracker_.add_measurements<ROSVec2fAccess>(src->positions, src->velocities, src->id);
      else if (!src->has_velocity)
        tracker_.add_measurements<ROSVec2fAccess>(src->positions, src->id);
    }
    else if (src->dimensionality == 3)
    {
      if (src->has_velocity)
        tracker_.add_measurements<ROSVec3fAccess>(src->positions, src->velocities, src->id);
      else if (!src->has_velocity)
        tracker_.add_measurements<ROSVec3fAccess>(src->positions, src->id);
    }

    // Keep track of how many measurements there are for utilization
    util_.number_of_rransac_measurements += src->positions.size();
  }


  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  std::vector<rransac::core::ModelPtr> tracks = tracker_.run();

  // Update how long it took librransac to run, used for utilization
  t_rransac_ = (ros::Time::now() - tic).toSec();


  // publish the tracks onto ROS network
  publish_tracks(tracks);

  // generate visualization only, but if someone is listening
  if (pub_tracks_video.getNumSubscribers() > 0)
    draw_tracks(tracks);
}

// ----------------------------------------------------------------------------

void RRANSAC::callback_video(const sensor_msgs::ImageConstPtr& frame, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
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

  // convert message data into OpenCV type cv::Mat
  frame_ = cv_bridge::toCvCopy(frame, "bgr8")->image;

  // Grab which sequence in the message stream this is (seq is deprecated)
  frame_seq_ = frame->header.seq;

  // Estimate fps
  // Update saved headers
  header_frame_last_ = header_frame_;
  header_frame_ = frame->header;

  // LPF alpha: Converge quickly at first
  double alpha = (frame_seq_ < 30) ? 0.95 : alpha_;

  // enforce realistic time differences (for rosbag looping)
  ros::Duration elapsed = header_frame_.stamp - header_frame_last_.stamp;
  if (!(elapsed.toSec()<=0 || elapsed.toSec()>1))
    fps_ = alpha*(1/elapsed.toSec()) + (1-alpha)*fps_;
}

// ----------------------------------------------------------------------------

void RRANSAC::callback_stats(const visual_mtt::Stats& data)
{
  // Save stride
  frame_stride_ = data.stride;
  double t_available = (1/fps_)*frame_stride_;

  // LPF alpha: Converge quickly at first
  double alpha = (frame_seq_ < 30) ? 0.95 : 1/(time_constant_/t_available + 1);

  util_.time_available          = t_available;
  util_.feature_manager         = alpha*(data.t_feature_manager/t_available) + (1-alpha)*util_.feature_manager;
  util_.homography_manager      = alpha*(data.t_homography_manager/t_available) + (1-alpha)*util_.homography_manager;
  util_.measurement_generation  = alpha*(data.t_measurement_generation/t_available) + (1-alpha)*util_.measurement_generation;
  util_.rransac                 = alpha*(t_rransac_/t_available) + (1-alpha)*util_.rransac;

  double total = util_.feature_manager + util_.homography_manager + util_.measurement_generation + util_.rransac;
  util_.total = alpha*total + (1-alpha)*util_.total;
}

// ----------------------------------------------------------------------------

void RRANSAC::publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
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

  // Attach utilization statistics to the message
  msg.util = util_;

  // ROS publish
  pub_tracks.publish(msg);
}

// ----------------------------------------------------------------------------

void RRANSAC::draw_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
{

  if (frame_.empty())
    return;

  cv::Mat draw = frame_.clone();

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
    cv::putText(draw, ssGMN.str().c_str(), cv::Point(center.x + 5, center.y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.85*text_scale_, cv::Scalar(0, 0, 180), 2);

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

  sprintf(text, "Frame: %d", frame_seq_);
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
  cv::Scalar background = (util_.total > 0.9) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255);

  sprintf(text, "Utilization: %d%%", (int)(util_.total*100));
  corner += corner_offset;
  cv::rectangle(draw, corner, corner + bl_corner, background, -1);
  cv::putText(draw, text, corner + text_offset, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));

  // Resize the image according to the scale
  cv::Mat resized;
  cv::Size size(draw.cols*pub_scale_, draw.rows*pub_scale_);
  cv::resize(draw, resized, size, 0, 0, cv::INTER_AREA);

  // Publish over ROS network
  cv_bridge::CvImage image_msg;
  image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  image_msg.image = resized;
  image_msg.header = header_frame_;
  pub_tracks_video.publish(image_msg.toImageMsg());
}

}
