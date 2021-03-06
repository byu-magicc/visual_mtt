#include "visual_frontend/measurement_manager/feature_motion.h"

namespace visual_frontend {

FeatureMotion::FeatureMotion()
{
  
  enabled_ = false;
  name_ = "Feature Motion";
  has_velocity_ = true;
  drawn_ = false;
  sigmaR_pos_ = 0.01;
  sigmaR_vel_=0.03;
  source_parameters_changed_ = false;

  first_image_ = true;
  velocity_floor_ = 0.002;
  velocity_ceiling_ = 0.02;
  pic_params_.pic_num = 0;
  pic_params_.file_name = "Feature_Motion";

#if TRACKING_SE2
  source_parameters_.type_ = rransac::MeasurementTypes::SEN_POS_VEL;
#else // R2
  source_parameters_.type_ = rransac::MeasurementTypes::RN_POS_VEL;  
#endif
  
  source_parameters_.meas_cov_ = Eigen::Matrix<double,4,4>::Identity();
  source_parameters_.meas_cov_.diagonal() << pow(sigmaR_pos_,2), pow(sigmaR_pos_,2), pow(sigmaR_vel_,2), pow(sigmaR_vel_,2);
  source_parameters_.spacial_density_of_false_meas_ = 0.01;
  source_parameters_.probability_of_detection_ = 0.95;
  source_parameters_.gate_threshold_ = 0.1;

// Required frames for plugin
#if OPENCV_CUDA
  frames_required_ = {false, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, false, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#else
  frames_required_ = {false, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
#endif
}

// ----------------------------------------------------------------------------

FeatureMotion::~FeatureMotion()
{
    DestroyWindows();
}

// ----------------------------------------------------------------------------
    
void FeatureMotion::Initialize(const common::Params& params, const unsigned int source_index) {
  source_parameters_.source_index_ = source_index;
}
// ----------------------------------------------------------------------------

void FeatureMotion::SetParameters(const visual_mtt::visual_frontendConfig& config)
{
  velocity_floor_ = config.minimum_feature_velocity;
  velocity_ceiling_ = config.maximum_feature_velocity;

  // noise parameters (only for storage, not used in measurement generation)
  if ((sigmaR_pos_ != config.feature_motion_sigmaR_pos || sigmaR_vel_ != config.feature_motion_sigmaR_vel) && source_parameters_.meas_cov_.rows() !=0 || source_parameters_.gate_threshold_!=config.RRANSAC_gate_threshold ) {
      sigmaR_pos_ = config.feature_motion_sigmaR_pos;
      sigmaR_vel_ = config.feature_motion_sigmaR_vel;
      source_parameters_.gate_threshold_ = config.RRANSAC_gate_threshold;
      source_parameters_.meas_cov_.diagonal() << pow(sigmaR_pos_,2), pow(sigmaR_pos_,2), pow(sigmaR_vel_,2), pow(sigmaR_vel_,2);
      source_parameters_changed_ = true;
  }
    

  ShouldReset(config.feature_motion_enabled);
}

// ----------------------------------------------------------------------------

void FeatureMotion::DrawMeasurements(const common::System& sys)
{
  cv::Mat draw = sys.GetFrame(common::SD).clone();

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> meas_pos_h, meas_pos_h_p; // homogeneous
  std::vector<cv::Point2f> meas_pos_d, meas_pos_d_p; // distorted
  if (meas_pos_.size()>0)
  {
    cv::convertPointsToHomogeneous(meas_pos_, meas_pos_h);
    cv::projectPoints(meas_pos_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sys.sd_camera_matrix_, sys.dist_coeff_, meas_pos_d);
  }
  if (meas_pos_parallax_.size() > 0)
  {
    cv::convertPointsToHomogeneous(meas_pos_parallax_, meas_pos_h_p);
    cv::projectPoints(meas_pos_h_p, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sys.sd_camera_matrix_, sys.dist_coeff_, meas_pos_d_p);
  }


  char text[40];
  cv::Point bl_corner = cv::Point(165, 18)*text_scale_;
  cv::Point corner_offset = cv::Point(0, 20)*text_scale_;
  cv::Point corner_offset2 = cv::Point(20, 0)*text_scale_;
  cv::Point text_offset = cv::Point(5, 13)*text_scale_;
  double text_size = 0.5*text_scale_;

  sprintf(text, "Moving:");
  cv::Point corner = cv::Point(5,5)*text_scale_;
  cv::rectangle(draw, corner, corner + bl_corner, cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + text_offset+corner_offset2, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));
  cv::circle(draw, corner + corner_offset/2 + corner_offset2/2, 2, cv::Scalar(205, 0, 0), 2, CV_AA);

  sprintf(text, "Parallax: ");
  corner += corner_offset;
  cv::rectangle(draw, corner, corner + bl_corner, cv::Scalar(255, 255, 255), -1);
  cv::putText(draw, text, corner + text_offset+corner_offset2, cv::FONT_HERSHEY_SIMPLEX, text_size, cv::Scalar(0, 0, 0));
  cv::circle(draw, corner + corner_offset/2 + corner_offset2/2 , 2, cv::Scalar(0, 0, 205), 2, CV_AA);

  // Draw the points moving along the epipolar lines
  for (int j=0; j<meas_pos_d_p.size(); j++)
  {
      cv::circle(draw, meas_pos_d_p[j], 2, cv::Scalar(0, 0, 205), 2, CV_AA);
  }

  // Draw the points moving perpendicular to the epipolar lines
  for (int jj=0; jj<meas_pos_d.size(); jj++)
  {
      cv::circle(draw, meas_pos_d[jj], 2, cv::Scalar(205, 0, 0), 2, CV_AA);
  }

  if(drawn_ == false)
  {
    cv::namedWindow(name_);
    cv::setMouseCallback(name_,sys.TakePicture, &pic_params_);
  }

  if (!draw.empty())
  {
    drawn_ = true;
    cv::imshow(name_, draw);
    pic_params_.img = draw.clone();
  }

}

// ----------------------------------------------------------------------------

bool FeatureMotion::GenerateMeasurements(const common::System& sys)
{
  bool good_measurements = false;
  meas_pos_.clear();
  meas_vel_.clear();
  meas_pos_parallax_.clear();

  double dt = sys.current_time_ - sys.prev_time_;


  // If there isn't a good transform, dont' do anything. 
  if (!first_image_ && sys.good_transform_)
  {

      if (dt <=0 ) {
        ROS_DEBUG_STREAM_THROTTLE(sys.message_output_period_, "The current image time stamp minus the previous image time stamp is less than or equal to 0. Setting it to 1. ");
        dt = 1;
      } 

    // Warp previous features forwards. All static features from sequential
    // frames will be aligned. Features from moving objects will be offset by
    // some distance, a velocity ("normalized image units" per frame).
    std::vector<cv::Point2f> corrected_pts;
    if (sys.ud_prev_matched_.size() > 0)
      cv::perspectiveTransform(sys.ud_prev_matched_, corrected_pts, sys.transform_);

    // Find the point velocities
    std::vector<cv::Point2f> meas_vel;
    for (int i = 0; i < corrected_pts.size(); ++i)
      meas_vel.push_back( (sys.ud_curr_matched_.at(i) - corrected_pts[i])/dt);

    // Save points whose disparity exceed the velocity threshold
    int numberOfPossibleMovers = 0;
    for (int ii = 0; ii < corrected_pts.size(); ++ii)
    {
      float vel = sqrt(meas_vel[ii].x*meas_vel[ii].x + meas_vel[ii].y*meas_vel[ii].y);
      if (vel > velocity_floor_ && vel < velocity_ceiling_)
      {
        if(sys.moving_parallax_[ii])
        {
          numberOfPossibleMovers++;
          meas_pos_.push_back(sys.ud_curr_matched_[ii]);
          meas_vel_.push_back(meas_vel[ii]);
        }
        else
        {
          meas_pos_parallax_.push_back(sys.ud_curr_matched_[ii]);
        }

      }
    }
  } else {
    first_image_ = false;
  }

  if (meas_pos_.size() > 0)
    good_measurements = true;
  else
    ROS_DEBUG_STREAM_THROTTLE(sys.message_output_period_,"FeatureMotion: No measurements found!");

  return good_measurements;

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void FeatureMotion::Reset() {

  DestroyWindows();
  drawn_ = false;
  first_image_ = true;


}

// ----------------------------------------------------------------------------

void FeatureMotion::DestroyWindows() {

  if (drawn_)
    cv::destroyWindow(name_);
}

}


// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::FeatureMotion, visual_frontend::MeasurementBase)