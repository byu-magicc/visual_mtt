#include "visual_frontend/measurement_manager/feature_motion.h"

namespace visual_frontend {

FeatureMotion::FeatureMotion()
{
  enabled_ = false;
  name_ = "Feature Motion";
  id_ = 0;
  has_velocity_ = true;
  drawn_ = false;

  first_image_ = true;
  velocity_floor_ = 0.002;
  velocity_ceiling_ = 0.02;

// Required frames for plugin
#if OPENCV_CUDA
  frames_required_ = {false, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, false, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#else
  frames_required_ = {false, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, false, false, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, _CUDA, HSV_CUDA}
#endif
}

// ----------------------------------------------------------------------------

FeatureMotion::~FeatureMotion()
{
    DestroyWindows();
}

// ----------------------------------------------------------------------------
    
void FeatureMotion::Initialize(const common::Params& params) {}

// ----------------------------------------------------------------------------

void FeatureMotion::SetParameters(const visual_mtt::visual_frontendConfig& config)
{
  velocity_floor_ = config.minimum_feature_velocity;
  velocity_ceiling_ = config.maximum_feature_velocity;

  // noise parameters (only for storage, not used in measurement generation)
  sigmaR_pos_ = config.feature_motion_sigmaR_pos;
  sigmaR_vel_ = config.feature_motion_sigmaR_vel;

  ShouldReset(config.feature_motion_enabled);
}

// ----------------------------------------------------------------------------

void FeatureMotion::DrawMeasurements(const common::System& sys)
{
  cv::Mat draw = sys.GetFrame(common::SD).clone();

  // treat points in the normalized image plane as 3D points (homogeneous).
  // project the points onto the sensor (pixel space) for plotting.
  // use no rotation or translation (world frame = camera frame).
  std::vector<cv::Point3f> meas_pos_h; // homogeneous
  std::vector<cv::Point2f> meas_pos_d; // distorted
  if (meas_pos_.size()>0)
  {
    cv::convertPointsToHomogeneous(meas_pos_, meas_pos_h);
    cv::projectPoints(meas_pos_h, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), sys.sd_camera_matrix_, sys.dist_coeff_, meas_pos_d);
  }

  // plot measurements
  for (int j=0; j<meas_pos_d.size(); j++)
  {
    cv::circle(draw, meas_pos_d[j], 2, cv::Scalar(255, 0, 255), 2, CV_AA);
  }

  if (!draw.empty())
  {
    drawn_ = true;
    cv::imshow(name_, draw);
  }

}

// ----------------------------------------------------------------------------

bool FeatureMotion::GenerateMeasurements(const common::System& sys)
{
  bool good_measurements = false;
  meas_pos_.clear();
  meas_vel_.clear();


  // If there isn't a good transform, dont' do anything. 
  if (!first_image_ && sys.good_transform_)
  {
    // Warp previous features forwards. All static features from sequential
    // frames will be aligned. Features from moving objects will be offset by
    // some distance, a velocity ("normalized image units" per frame).
    std::vector<cv::Point2f> corrected_pts;
    if (sys.ud_prev_matched_.size() > 0)
      cv::perspectiveTransform(sys.ud_prev_matched_, corrected_pts, sys.transform_);

    // Find the point velocities
    std::vector<cv::Point2f> meas_vel;
    for (int i = 0; i < corrected_pts.size(); ++i)
      meas_vel.push_back(sys.ud_curr_matched_.at(i) - corrected_pts[i]);

    // Save points whose disparity exceed the velocity threshold
    int numberOfPossibleMovers = 0;
    for (int ii = 0; ii < corrected_pts.size(); ++ii)
    {
      float vel = sqrt(meas_vel[ii].x*meas_vel[ii].x + meas_vel[ii].y*meas_vel[ii].y);
      if (vel > velocity_floor_ && vel < velocity_ceiling_)
      {
        numberOfPossibleMovers++;
        meas_pos_.push_back(sys.ud_curr_matched_[ii]);
        meas_vel_.push_back(meas_vel[ii]);
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