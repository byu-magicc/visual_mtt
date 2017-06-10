#include "rransac/rransac.h"

namespace visual_mtt {

RRANSAC::RRANSAC()
{
  // TODO: This is a hack. We need to set the surveilance region width/height here.
  // this is something to think about in the context of normalized image coordinates
  params_.frame_cols = 1000;
  params_.frame_rows = 1000;

  // get parameters from param server that are not dynamically reconfigurable
  // TODO: is it worth it to use a private node handle here?
  nh.param<bool>("rransac/show_tracks", show_tracks_, 0);

  // instantiate the rransac::Tracker library class
  tracker_ = rransac::Tracker(params_);

  // ROS stuff
  image_transport::ImageTransport it(nh);
  sub_video = it.subscribe("video", 1, &RRANSAC::callback_video, this);
  sub_scan = nh.subscribe("measurements", 1, &RRANSAC::callback_scan, this);
  pub = nh.advertise<visual_mtt2::Tracks>("tracks", 1);

  // establish dynamic reconfigure and load defaults
  auto func = std::bind(&RRANSAC::callback_reconfigure, this, std::placeholders::_1, std::placeholders::_2);
  server_.setCallback(func);

  // populate plotting colors
  colors_ = std::vector<cv::Scalar>();
  for (int i = 0; i < 1000; i++)
    colors_.push_back(cv::Scalar(std::rand() % 256, std::rand() % 256, std::rand() % 256));
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void RRANSAC::callback_reconfigure(visual_mtt2::rransacConfig& config, uint32_t level) {

  // general
  params_.dt = config.dt;

  // R-RANSAC specific parameters
  params_.Nw = config.Nw;
  params_.M = config.M;
  params_.tauR = config.tauR;
  params_.sigmaR_pos = config.sigmaR_pos;
  params_.sigmaR_vel = config.sigmaR_vel;
  params_.set_motion_model(static_cast<enum rransac::core::MotionModelType>(config.rransac_motion_model));

  // RANSAC specific parameters
  params_.ell = config.ell;
  params_.guided_sampling_threshold = config.guided_sampling_threshold;
  params_.tauR_RANSAC = config.tauR_RANSAC;
  params_.gamma = config.gamma;
  params_.sigmaR_pos_RANSAC = config.sigmaR_pos_RANSAC;
  params_.set_motion_model_RANSAC(static_cast<enum rransac::core::MotionModelType>(config.ransac_motion_model));

  // model merging parameters
  params_.tau_vel_percent_diff = config.tau_vel_percent_diff;
  params_.tau_vel_abs_diff = config.tau_vel_abs_diff;
  params_.tau_angle_abs_diff = config.tau_angle_abs_diff;
  params_.tau_xpos_abs_diff = config.tau_xpos_abs_diff;
  params_.tau_ypos_abs_diff = config.tau_ypos_abs_diff;

  // model pruning parameters
  params_.frame_cols = config.frame_cols;
  params_.frame_rows = config.frame_rows;
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

void RRANSAC::callback_scan(const visual_mtt2::RRANSACScanPtr& rransac_scan)
{
  // Save the original frame header
  header_frame_ = rransac_scan->header;

  // Access the homography from the ROS message, convert to Projective2d, and give to R-RANSAC
  Eigen::Matrix3f H = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(rransac_scan->homography.data());
  Eigen::Projective2d T(H.cast<double>());
  tracker_.apply_transformation(T);


  // Add each source's measurements along with its ID to the R-RANSAC Tracker
  for (auto src = rransac_scan->sources.begin(); src != rransac_scan->sources.end(); src++)
    if (src->dimensionality == 2)
      tracker_.add_measurements<ROSVec2fAccess>(src->positions, src->velocities, src->id);
    else if (src->dimensionality == 3)
      tracker_.add_measurements<ROSVec3fAccess>(src->positions, src->velocities, src->id);


  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  std::vector<rransac::core::ModelPtr> tracks = tracker_.run();


  // publish the tracks onto ROS network
  publish_tracks(tracks);

  // generate visualization
  if (show_tracks_)
    draw_tracks(tracks);
}

// ----------------------------------------------------------------------------

void RRANSAC::callback_video(const sensor_msgs::ImageConstPtr& frame)
{
  // convert message data into OpenCV type cv::Mat
  frame_ = cv_bridge::toCvCopy(frame, "bgr8")->image;

  // Grab which sequence in the message stream this is (seq is deprecated)
  frame_seq_ = frame->header.seq;
}

// ----------------------------------------------------------------------------

void RRANSAC::publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
{

  // Create the ROS message we will send
  visual_mtt2::Tracks msg;

  for (int i=0; i<tracks.size(); i++)
  {
    visual_mtt2::Track track;

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

  // ROS publish
  pub.publish(msg);
}

// ----------------------------------------------------------------------------

void RRANSAC::draw_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
{
  cv::Mat draw = frame_.clone();

  int total_tracks = 0;

  for (int i=0; i<tracks.size(); i++)
  {
    total_tracks = std::max(total_tracks, (int)tracks[i]->GMN);
    cv::Scalar color = colors_[tracks[i]->GMN];

    // draw circle with center at position estimate
    cv::Point center;
    center.x = tracks[i]->xhat(0);
    center.y = tracks[i]->xhat(1);
    cv::circle(draw, center, (int)params_.tauR, color, 2, 8, 0);

    // draw red dot at the position estimate
    cv::circle(draw, center, 2, cv::Scalar(0, 0, 255), 2, 8, 0); // TODO: make 2 (radius) a param

    // draw scaled velocity vector
    cv::Point velocity;
    double velocity_scale = 10; // TODO: adjust after switch to normalized image coordinates
    velocity.x = tracks[i]->xhat(2) * velocity_scale;
    velocity.y = tracks[i]->xhat(3) * velocity_scale;
    cv::line(draw, center, center + velocity, color, 1, CV_AA);

    // draw model number and inlier ratio
    std::stringstream ssGMN;
    ssGMN << tracks[i]->GMN << ", " << tracks[i]->rho;
    cv::putText(draw, ssGMN.str().c_str(), cv::Point(center.x + 5, center.y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

    // draw covariance (?)

    // draw consensus sets
    for (int j=0; j<tracks[i]->CS.size(); j++)
    {
      center.x = tracks[i]->CS[j]->pos(0);
      center.y = tracks[i]->CS[j]->pos(1);
      cv::circle(draw, center, 2, color, -1, 8, 0); // TODO: make 2 (radius) a param
    }
  }

  // draw top-left box
  char text[40];

  sprintf(text, "Frame %d", frame_seq_);
  cv::Point corner = cv::Point(10,2);
  cv::rectangle(draw, corner, corner + cv::Point(165, 18), cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + cv::Point(5, 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  sprintf(text, "Total models: %d", total_tracks);
  corner = cv::Point(10,22);
  cv::rectangle(draw, corner, corner + cv::Point(165, 18), cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + cv::Point(5, 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  sprintf(text, "Current models:  %d", (int)tracks.size());
  corner = cv::Point(10,42);
  cv::rectangle(draw, corner, corner + cv::Point(165, 18), cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + cv::Point(5, 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  cv::imshow("Tracks", draw);
  cv::waitKey(1);
}

}