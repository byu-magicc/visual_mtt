#include "visual_frontend/transform_manager/simple_homography.h"

namespace visual_frontend {

// ----------------------------------------------------------------------------

SimpleHomography::SimpleHomography()
{
  enabled_ = false;
  drawn_ = false;
  name_ = "Simple Homography";
  pic_params_.pic_num = 0;
  pic_params_.file_name = "Simple_Homography";

// Required frames for plugin
#if OPENCV_CUDA
  frames_required_ = {false, false, false, false, false};  // {HD, SD, MONO, UNDIST, HSV}
  cuda_frames_required_ = {false, false, false, true, false};  // {HD_CUDA, SD_CUDA, MONO_CUDA, UNDIST_CUDA, HSV_CUDA}
#else
  frames_required_ = {false, false, false, true, false};  // {HD, SD, MONO, UNDIST, HSV}
#endif
}

// ----------------------------------------------------------------------------

SimpleHomography::~SimpleHomography(){

  DestroyWindows();
}

// ----------------------------------------------------------------------------

void SimpleHomography::Initialize(const common::Params& params){}

// ----------------------------------------------------------------------------

void SimpleHomography::SetParameters(const visual_mtt::visual_frontendConfig& config) 
{
  ShouldReset(config.simple_homography_enabled);
}

// ----------------------------------------------------------------------------

void SimpleHomography::DrawTransform(const common::System& sys){

  cv::Size frame_size = sys.GetFrame(common::SD).size();

  if (!frame_u_last_.empty())
  {
    cv::Mat camera_matrix;

    // convert the euclidean homography to pixel homography
    sys.sd_camera_matrix_.convertTo(camera_matrix, CV_32FC1); // needed for inverse
    cv::Mat transform_pixel = camera_matrix*transform_*camera_matrix.inv();

  #if OPENCV_CUDA

    // transform previous image using new homography
    cv::cuda::GpuMat frame_u_last_warped;
    cv::cuda::warpPerspective(frame_u_last_, frame_u_last_warped, transform_pixel, frame_size);

    // raw difference
    cv::cuda::absdiff(sys.GetCUDAFrame(common::UNDIST_CUDA), frame_u_last_warped, frame_difference_);

  #else

    // transform previous image using new homography
    cv::Mat frame_u_last_warped;
    cv::warpPerspective(frame_u_last_, frame_u_last_warped, transform_pixel, frame_size);

    // raw difference
    cv::absdiff(sys.GetFrame(common::UNDIST), frame_u_last_warped, frame_difference_);

  #endif

  #if OPENCV_CUDA
    cv::Mat frame_difference(frame_difference_);
    cv::imshow(name_, frame_difference);
    pic_params_.img = frame_difference.clone();
  #else
    cv::imshow(name_, frame_difference_);
    pic_params_.img = frame_difference_.clone();    
  #endif
  }

  if(drawn_ == false)
  {
    cv::namedWindow(name_);
    cv::setMouseCallback(name_,sys.TakePicture, &pic_params_);
  }

  drawn_ = true;

  // age undistorted image (save, overwriting the old one)
#if OPENCV_CUDA
  frame_u_last_ = sys.GetCUDAFrame(common::UNDIST_CUDA).clone();
#else
  frame_u_last_ = sys.GetFrame(common::UNDIST).clone();
#endif


}

// ----------------------------------------------------------------------------

bool SimpleHomography::GetTransform(const common::System& sys)
{
  good_transform_ = false;

  // If there aren't enough feature pairs to find a homography, just bail and let
  // other people know there isn't a transform to be trusted.
  if (!sys.good_features_)
  {
    transform_ = cv::Mat::eye(3, 3, CV_32F);
  } else {

    // calculate the homography
    const double reprojection_error = 0.001; // the optimization aims to minimize below this value
    transform_ = cv::findHomography(sys.ud_prev_matched_, sys.ud_curr_matched_, CV_RANSAC, reprojection_error, inlier_mask_);

    // check that the homography is not empty
    if (transform_.empty())
    {
      transform_ = cv::Mat::eye(3, 3, CV_32F);
    } else {
    // baptize the homography
    transform_.convertTo(transform_, CV_32F);


    }


    

    // use inlier count to determine if the homography is good
    int inlier_count = 0;
    for(int i=0; i<inlier_mask_.size(); i++)
      inlier_count += inlier_mask_[i] ? 1 : 0;

    // TODO: should this (20) be a % of the pairs rather than a set value?
    if (inlier_count < 20)
    {
      ROS_WARN_STREAM_THROTTLE(sys.message_output_period_,"(" << "#" << ") " << "homography calculator: few homography inliers (" << inlier_count << ")");
      // TODO: replace # with frame number
    }
    else
    {
      good_transform_ = true;
    }
  }

  return good_transform_;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void SimpleHomography::Reset() 
{

  DestroyWindows();
  drawn_ = false;

}

void SimpleHomography::DestroyWindows() 
{

  if(drawn_)
    cv::destroyWindow(name_);
}



//-----------------------------------------------------------------------------
void SimpleHomography::ProjectHomographySE2() {

  // The intrinsic camera parameters for the normalized and undistorted points.
  cv::Mat K = cv::Mat::eye(3,3,CV_32F);
  std::vector<cv::Mat> rotations, translations, normals;
  std::vector<cv::Mat> p_r, p_t, p_n;
  std::vector<cv::Mat> Hps;
  cv::Mat V;
  cv::Mat R;
  cv::Mat Hp;
  cv::Mat Hp_best;
  cv::Mat U = cv::Mat::zeros(3,1,CV_64F);
  cv::Mat N = cv::Mat::zeros(3,1,CV_64F);
  cv::Mat T;
  transform_.convertTo(T, CV_64F); 
  int solutions;
  

  solutions = cv::decomposeHomographyMat(transform_,K,rotations,translations,normals);
  std::cout << "solutions: " << solutions << std::endl;
  std::cerr << std::endl << "transform_" << std::endl << transform_ << std::endl;

  // There is only a rotation
  if (solutions ==1) {
    cv::Rodrigues(rotations[0],V);
    U.at<double>(2) = V.at<double>(2); // extract the z component;
    cv::Rodrigues(U,R);
    R.convertTo(transform_, CV_32F); 
    

  } else {

    for (int ii = 0; ii < normals.size(); ++ii) {

      // std::cout << std::endl << "normals" << std::endl << normals[ii].type() << std::endl;
      // std::cout << std::endl << "normals" << std::endl << normals[ii].at<double>(2) << std::endl;
      // std::cout << std::endl << "translations" << std::endl << translations[ii] << std::endl;
      // std::cout << std::endl << "rotations" << std::endl << rotations[ii].type() << std::endl;


      if (normals[ii].at<double>(2) > 0) {
        // std::cout << "blah: " << std::endl;
        p_r.push_back(rotations[ii]);
        p_t.push_back(translations[ii]);
      }
    }

    // std::cout << "here1 " << std::endl;


    

    double best_score = 1e9;
    double n = 0;;
    
    N.at<double>(2) = 1;
    // std::cerr << "size pr " << std::endl << p_r.size() << std::endl;
    for (int ii =0; ii < p_t.size(); ++ii) {

      cv::Rodrigues(p_r[ii],V);
      // std::cout << "here3 " << std::endl;

      U.at<double>(2) = V.at<double>(2); // extract the z component;
      cv::Rodrigues(U,R);
      // std::cout << "here4 " << std::endl;

      // std::cout << std::endl << "rotations" << std::endl << R << std::endl;
      // std::cout << std::endl << "normals" << std::endl << N << std::endl;
      // std::cout << std::endl << "translations" << std::endl << p_t[ii] << std::endl;

      Hp = R + p_t[ii]*N.t();
      // std::cout << "Hp " << std::endl << Hp << std::endl;

      n = cv::norm(T - Hp);
      if (n < best_score) {
        Hp_best = Hp.clone();
        best_score = n;
      }
      
    }
    // std::cout << "here5 " << std::endl;

    Hp_best.convertTo(transform_, CV_32F);   


  }

  std::cerr << std::endl << "transform_" << std::endl << transform_ << std::endl;




}

} //namespace visual_frontend

// Macro needed to register the class. This macro helps with 
// name mangling so that it can be imported dynamically.
PLUGINLIB_EXPORT_CLASS(visual_frontend::SimpleHomography, visual_frontend::TransformBase)