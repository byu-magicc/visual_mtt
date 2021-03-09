#pragma once

#include <common/gpu.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <ros/console.h>

#include <common/opencv_compat.h>

#include "common/object_types.h"

namespace common {

/** \struct Camera Pose
 * The pose of the camera is represented by a position and a 
 * Quaternion component. The position is expressed in the inertial with respect to the inertial frame.
 * The Quaternion should be the rotation from the camera's body frame to the inertial frame. 
 */
struct CameraPose {

Eigen::Matrix<double,3,1> P;       /**< The postion of the camera expressed in the inertial frame w.r.t. the camera frame. */
Eigen::Quaternion<double> Q;       /**< The orientation of the camera frame that denotes the rotation from the camera frame to the inertial frame. */
Eigen::Quaternion<double> Q_c_bl;  /**< The rotation from the camera frame to the body level frame. This rotation allows us to rotate the measurements to the normalized virtual image plane (NVIP). */
double time_stamp;                 /**< The time stamp in seconds of the camera pose. */
}; 


/** \struct PictureParams
* \brief Parameters needed to save an image
* \detail When a user double clicks on an OpenCV window under tuning mode,
*         an image will be saved according to the name given.
* @see System::picture_file_path_
* @see System::TakePicture
*/
struct PictureParams
{

  cv::Mat img;            /**< The image that will be saved */
  std::string file_name;       /**< The name of the image, not the filepath. */
  unsigned pic_num = 0;   /**< The Picture number. Used for naming. Ex: name_<pic_num>.png */

};

const int num_frame_types_ = 5;               /**< The total number of possible frames types available for request. */
enum frame_type_ {HD, SD, MONO, UNDIST, HSV}; /**< All frames type names available for request. */
#if OPENCV_CUDA
const int num_cuda_frame_types_ = 5;                                        /**< The total number of possible CUDA frames types available for request. */
enum frame_type_cuda_ {HD_CUDA, SD_CUDA, MONO_CUDA, UNDIST_CUDA, HSV_CUDA}; /**< All CUDA frames type names available for request. */
typedef std::array<bool, num_cuda_frame_types_> CUDAFrameRefVector; /**< Data type defining boolean references to associated CUDA frame types. Used for frame requests and existence checks. */
#endif

typedef std::array<bool, num_frame_types_> FrameRefVector;          /**< Data type defining boolean references to associated frame types. Used for frame requests and existence checks. */

/** \class System
 * \brief Handles all the data needed for the different managers, plugins, visual_frontend::VisualFrontend, and RRANSAC.
 * \detail Handles all the data needed for the different managers and plugins and provides basic methods to 
 * facilitate the exchange and sharing of data between different managers, plugins, RRANSAC, and visual_frontend::VisualFrontend.
 * All plugins have access to the data, but cannot write to it. This is to prevent a plugin from changing
 * data that they shouldn't. The managers act at the gate keepers of writing information from the plugins to System so that it 
 * can be shared with the rest of the program.
 */

class System {

  public:

  System();
  ~System();

  /**
  * \breif Sets System::hd_frame_.
  * @param hd_frame is the new unaltered image.
  * @see System::hd_frame_
  */
  void SetHDFrame(const cv::Mat& hd_frame);

  /**
  * \breif Sets System::resize_scale_.
  * @param resize_scale scales the current hd_frame.
  * @see System::resize_scale_
  */
  void SetResizeScale(const double resize_scale);

  /**
  * \breif Sets System::hd_camera_matrix_ and System::dist_coeff_.
  * @param K is the camera matrix.
  * @param D is the distortion coefficients.
  * @see System::hd_camera_matrix_
  * @see System::dist_coeff_
  */
  void SetCameraParams(const cv::Mat& K, const cv::Mat& D);

  /**
  * \breif Sets System::sd_camera_matrix_.
  * \detial This is called whenever the size of System::sd_frame_ changes.
  * When the size changes, the System::undistorted_region_mask_
  * needs to be updated according to the new size. To do this
  * it calls the member method System::SetUndistortedRegionMask().
  * @see System::sd_camera_matrix_
  * @see System::SetUndistortedRegionMask()
  */
  void SetScaledCameraParams();

  /**
  * \breif  Sets System::undistorted_region_mask_.
  * \detial This method is called by System::SetScaledCameraParams()
  * whenever the size of System::sd_frame_ changes to update 
  * the new undistroted region mask.
  * @see System::undistorted_region_mask_
  */
  void SetUndistortedRegionMask();

  /**
  * \breif Clears several flags by setting them to false. 
  * \detial Clears System::good_features_, System::good_transform_, and System::good_measurements_.
  * @see System::good_features_
  * @see System::good_transform_
  * @see System::good_measurements_
  */
  void ClearFlags();

  //////////////////////////////////////////////////////////////////////////////////////
  // Feature Manager
  //////////////////////////////////////////////////////////////////////////////////////

  /**
  * 
  * \detail This method is called by visual_frontend::FeatureManager at the beginning
  * of each iteration to empty the contents of System::d_prev_matched_, System::d_curr_matched_,
  * System::ud_prev_matched_, and System::ud_curr_matched_. 
  * @see System::d_prev_matched_
  * @see System::d_curr_matched_
  * @see System::ud_prev_matched_
  * @see System::ud_curr_matched_
  * @see visual_frontend::FeatureManager::FindCorrespondences(common::System& sys)
  */
  void ClearMatchedFeatures();

  /**
  * 
  * \detail This method is called by visual_frontend::FeatureManager to add undistorted
  * matched features, generated by feature plugins, to System::d_prev_matched_, and 
  * System::d_curr_matched_.
  * @param d_prev_matched The matched features of the previous frame.
  * @param d_curr_matched The matched features of the current frame. 
  * @see System::d_prev_matched_
  * @see System::d_curr_matched_
  * @see visual_frontend::FeatureManager::FindCorrespondences(common::System& sys)
  */
  void AddMatchedFeatures(const std::vector<cv::Point2f>& d_prev_matched, const std::vector<cv::Point2f>& d_curr_matched);
  
  /**
  * 
  * \detail This method is called by visual_frontend::FeatureManager to undistort
  * the matched features (System::d_prev_matched_, System::d_curr_matched_)
  * and project them onto the normalized image plane. The undistorted and
  * normalized features are stored in System::ud_prev_matched_ and
  * System::ud_curr_matched_.
  * @see System::ud_prev_matched_
  * @see System::ud_curr_matched_
  * @see visual_frontend::FeatureManager::FindCorrespondences(common::System& sys)
  */
  void UndistortMatchedFeatures();

  /**
  * 
  * \detail This method is called by visual_frontend::FeatureManager to set
  * the flag System::good_features_.
  * @param good_features Flag indicates if the features are good.
  * @see System::good_features_
  * @see visual_frontend::TransformManager::CalculateTransform(common::System& sys).
  * @see visual_frontend::FeatureManager::FindCorrespondences(common::System& sys)
  */
  void SetFeatureFlag(const bool& good_features);

  //////////////////////////////////////////////////////////////////////////////////////
  // Transform Manager
  //////////////////////////////////////////////////////////////////////////////////////

  /**
  * 
  * \detail This method is called by visual_frontend::TransformManager at
  * the beginning of each iteration to set System::transform_ 
  * to the identity matrix.
  * @see System::transform_
  * @see visual_frontend::TransformManager::CalculateTransform(common::System& sys)
  */
  void ClearTransform();

  /**
  * 
  * \detail This method is called by visual_frontend::TransformManager once
  * a valid transform has been found. It sets System::transform_.
  * @param transform Transform generated from a Transform Method Plugin.
  * @see System::transform_
  * @see visual_frontend::TransformManager::CalculateTransform(common::System& sys)
  */
  void SetTransform(const cv::Mat& transform);

  /**
  * 
  * \detail This method is called by visual_frontend::TransformManager to
  * indicate if a good transform was found. It sets the flag System::good_transform_.
  * @param good_transform Flag indicates if the transform is good.
  * @see System::good_transform_
  * @see visual_frontend::TransformManager::CalculateTransform(common::System& sys)
  */
  void SetTransformFlag(const bool& good_transform);

  //////////////////////////////////////////////////////////////////////////////////////
  // Measurment Manager
  //////////////////////////////////////////////////////////////////////////////////////

  /**
  * 
  * \detail This method is called by visual_frontend::MeasurementManager at
  * the beginning of each iteration to clear System::measurements_.
  * @see System::measurements_
  * @see visual_frontend::MeasurementManager::GenerateMeasurements(common::System& sys)
  */
  void ClearMeasurements();

  /**
  * 
  * \detail This method is called by visual_frontend::MeasurementManager when
  * a Measurement Source Plugin generates measurements. These measurements
  * are added to System::measurements_.
  * @param id The Measurement Source Plugin's unique ID.
  * @param has_velocity Indicates if the Measurement Source Plugin generated velocity measurements (meas_vel)
  * @param meas_pos The 2D position of the measurement in undistorted normalized image pixel coordinates.
  * @param meas_vel The 2D velocity of the measurement in undistorted normalized image pixel coordinates.
  * @see visual_frontend::MeasurementBase::id_
  * @see visual_frontend::MeasurementBase::has_velocity_
  * @see common::Measurements::meas_pos
  * @see common::Measurements::meas_vel
  * @see visual_frontend::MeasurementManager::GenerateMeasurements(common::System& sys)
  */
  void AddMeasurements(const rransac::SourceParameters& source_params,
                       const bool has_velocity, 
                       const std::vector<cv::Point2f>& meas_pos, 
                       const std::vector<cv::Point2f>& meas_vel);

  /**
  * 
  * \detail This method is called by visual_frontend::MeasurementManager 
  * to indicate if any measurements were generated. This method
  * sets the flag System::good_measurements_ 
  * @param System::good_measurements_
  * @see visual_frontend::MeasurementManager::GenerateMeasurements(common::System& sys)
  */
  void SetMeasurementFlag(const bool& good_measurements);

  /**
  * 
  * \detail Sets the file path member variable, and the flag picture_file_path_set_.
  * @param  picture_file_path The file path to the directory where the images will be saved.
  */
  static void SetPictureFilepath(const std::string& picture_file_path);


  /**
  * 
  * \detail Callback for setMouseCallback. It is used to save pictures when the user
  *          double clicks an OpenCV image.
  * @param event Mouse event
  * @param x x position of the click
  * @param y y position of the click
  * @param void, contains the image and the name.
  */
  static void TakePicture(int event, int x, int y, int, void* param);

  /**
  * 
  * \detail This method acts as a generic getter function for any available frame type 
  * requested by the frame_type argument. Returns error message if frame has not been 
  * created at current iteration as indicated by common::System::frame_exists_.
  * @param frame_type The requested frame type.
  * @see common::frame_type_
  * @see common::System::frame_exists_
  */
  cv::Mat GetFrame(frame_type_ frame_type) const;

#if OPENCV_CUDA
  /**
  * 
  * \detail This method acts as a generic getter function for any available CUDA frame 
  * type requested by the frame_type_cuda argument. Returns error message if frame has 
  * not been created at current iteration as indicated by common::System::cuda_frame_exists_.
  * @param frame_type_cuda_ The requested CUDA frame type.
  * @see common::frame_type_cuda_
  * @see common::System::cuda_frame_exists_
  */
  cv::cuda::GpuMat GetCUDAFrame(frame_type_cuda_ frame_type_cuda) const;
#endif

  /**
  * 
  * \detail This method is called in the visual_frontend::VisualFrontend constructor to generate/update
  * all frames defined as required by common::System::frames_required_. The method will
  * set appropriate frame flags on common::System::frame_exists_ to indicate frame is 
  * available for access by common::System::GetFrame(frame_type_ frame_type).
  * @see common::System::frames_required_
  * @see common::System::frame_exists_
  * @see common::System::GetFrame(frame_type_ frame_type)
  * @see visual_frontend::VisualFrontend()
  */
  void SetFrames();

#if OPENCV_CUDA
  /**
  * 
  * \detail This method is called in the visual_frontend::VisualFrontend constructor to generate/update
  * all frames defined as required by common::System::cuda_frames_required_. The method will
  * set appropriate frame flags on common::System::cuda_frame_exists_ to indicate frame is 
  * available for access by common::System::GetCUDAFrame(frame_type_cuda_ frame_type_cuda).
  * @see common::System::cuda_frames_required_
  * @see common::System::cuda_frame_exists_
  * @see common::System::GetCUDAFrame(frame_type_cuda_ frame_type_cuda)
  * @see visual_frontend::VisualFrontend()
  */
  void SetCUDAFrames();
#endif

  /**
  * 
  * \detail This method is called by visual_frontend::MeasurementManager, 
  * visual_frontend::TransformManager, and visual_frontend::FeatureManager to add 
  * visual_frontend::MeasurementManager::frames_required_, 
  * visual_frontend::TransformManager::frames_required_, and
  * visual_frontend::FeatureManager::frames_required_ to common::System::frames_required_
  * indicating to common::System::SetFrames() which frames should be created at each iteration.
  * @param FrameRefVector& The requested frame set to be registered into common::System::frames_required_.
  * @see visual_frontend::MeasurementManager::frames_required_
  * @see visual_frontend::TransformManager::frames_required_
  * @see visual_frontend::FeatureManager::frames_required_
  * @see common::System::frames_required_
  * @see common::System::SetFrames()
  * @see visual_frontend::MeasurementManager
  * @see visual_frontend::TransformManager
  * @see visual_frontend::FeatureManager
  */
  void RegisterPluginFrames(FrameRefVector& plugin_frames);

#if OPENCV_CUDA
  /**
  * 
  * \detail This method is called by visual_frontend::MeasurementManager, 
  * visual_frontend::TransformManager, and visual_frontend::FeatureManager to add 
  * visual_frontend::MeasurementManager::cuda_frames_required_, 
  * visual_frontend::TransformManager::cuda_frames_required_, and
  * visual_frontend::FeatureManager::cuda_frames_required_ to common::System::cuda_frames_required_
  * indicating to common::System::SetCUDAFrames() which frames should be created at each iteration.
  * @param FrameRefVector& The requested frame set to be registered into common::System::cuda_frames_required_.
  * @see visual_frontend::MeasurementManager::cuda_frames_required_
  * @see visual_frontend::TransformManager::cuda_frames_required_
  * @see visual_frontend::FeatureManager::cuda_frames_required_
  * @see common::System::cuda_frames_required_
  * @see common::System::SetCUDAFrames()
  * @see visual_frontend::MeasurementManager
  * @see visual_frontend::TransformManager
  * @see visual_frontend::FeatureManager
  */
  void RegisterPluginCUDAFrames(CUDAFrameRefVector& plugin_frames_cuda);
#endif

  /**
  * 
  * \detail This method is called by visual_frontend::VisualFrontend to add 
  * the provided frame type to common::System::frames_required_
  * @param frame_type The requested frame type.
  * @see common::System::frames_required_
  */
  void RegisterFrame(frame_type_ frame_type);

#if OPENCV_CUDA
  /**
  * 
  * \detail This method is called by visual_frontend::VisualFrontend to add 
  * the provided frame type to common::System::cuda_frames_required_
  * @param frame_type_cuda_ The requested CUDA frame type.
  * @see common::System::cuda_frames_required_
  */
  void RegisterCUDAFrame(frame_type_cuda_ frame_type_cuda);
#endif

  /**
  * 
  * \detail Resets all flags in common::System::frame_exists_ and common::System::cuda_frame_exists_ to false
  * to indicate to system that frames must be updated at next iteration before requesting frames.
  * @see common::System::frame_exists_
  * @see common::System::cuda_frame_exists_
  */
  void ResetFrames();


  /**
   * Sets image_camera_pose_ to latest_camera_pose_ and computes the value CameraPose::Q_c_bl
   * which is the rotation from the camera to the body level frame of the UAV   * 
   */ 
  void SetImageCameraPose();

  /**
   * If the pose of the camera is provided, then this function will transform the points
   * from the normalized image plane to the virtual normalized image plane or the inverse
   * depending on the parameter inverse.
   * @param points The points that will be transformed
   * @param inverse If false, the points will be transformed from the image plane to the normalized image plane; otherise the inverse operation. 
   */ 
  std::vector<cv::Point2f> TransformBetweenImagePlanes(const std::vector<cv::Point2f> points, const bool inverse) const;

/**< */

  // Camera poses
  CameraPose latest_camera_pose_;        /**< The lastest provided camera pose. */
  CameraPose image_camera_pose_;         /**< The pose of the camera when an image is being processed. This is necessary since the rate at which we receive camera poses can be different than the rate we receive images. */
  bool camera_pose_available_=false;     /**< Indicates if the camera pose is available. This is set to true by the callback function that sets the latest_camera_pose_. */

  // Feature Manager
  std::vector<cv::Point2f> d_prev_matched_;  /**< Distorted matched features from the previous frame. @see visual_frontend::FeatureManager.*/
  std::vector<cv::Point2f> d_curr_matched_;  /**< Distorted matched features from the current frame. @see visual_frontend::FeatureManager.*/
  std::vector<cv::Point2f> ud_prev_matched_; /**< Undistorted and normalized matched features from the previous frame. @see visual_frontend::FeatureManager.*/
  std::vector<cv::Point2f> ud_curr_matched_; /**< Undistorted and normalized matched features from the current frame. @see visual_frontend::FeatureManager.*/
  bool good_features_;                       /**< Flag used to indicate that there are enough matched features to calculate a transform. @see visual_frontend::FeatureManager.*/              

  // Transform Manager
  cv::Mat transform_;                        /**< The transformation between the previous image and the current image. @see visual_frontend::TransformManager.*/
  bool good_transform_;                      /**< Flag used to indicate that the tranformation is good. @see visual_frontend::TransformManager.*/

  std::vector<bool> moving_parallax_;         /**< Flag indicating whether the point is moving perpendicular to epipolar lines (ie. whether it is an outlier to the Essential Matrix).*/
  
  // Measurement Manager
  std::list<rransac::Meas<double>> measurements_;   /**< Measurements produced by the Measurement sources. @see common::Measurements and visual_frontend::MeasurementManager. */
  int num_of_measurements_;                  /**< Total number of measurements produced by all of the measurement sources in one iteration. @see visual_frontend::MeasurementManager.*/
  bool good_measurements_;                   /**< Flag used to indicate if there are any measurements. @see visual_frontend::MeasurementManager.*/

  
  ////////////////////////////////////////////////////////////////////////
  // Visual Frontend
  // Images, camera info, and mask
  cv::Mat hd_frame_;                         /**< The unaltered image of the current frame. */ 
  cv::Mat sd_frame_;                         /**< Resized common::System::hd_frame_ image by common::System::resize_scale_. */
  cv::Mat mono_frame_;                       /**< Converted common::System::sd_frame_ image as greyscale image using CV_RGB2GRAY. */
  cv::Mat undist_frame_;                     /**< Converted common::System::sd_frame_ image as undistorted image using cv::undistort. */
  cv::Mat hsv_frame_;                        /**< Converted common::System::sd_frame_ image as HSV image using CV_BGR2HSV. */
  cv::Size sd_res_;                          /**< The image size of common::System::sd_frame_. */
  cv::Mat hd_camera_matrix_;                 /**< The intrinsic camera parameter's of common::System::hd_frame_. */
  cv::Mat sd_camera_matrix_;                 /**< The intrinsic camera parameter's of common::System::sd_frame_. */
  cv::Mat dist_coeff_;                       /**< The camera's distortion coefficients. */
  cv::Mat undistorted_region_mask_;          /**< Marks the region in System::sd_frame_ that is not removed when undistorted. When an
                                                  image is undistorted, parts of the image's border are removed if they extend beyond the original
                                                  size of the image. This mask indicates the region of the distorted image that will not be removed
                                                  when undistorted. */  
  FrameRefVector frames_required_;           /**< Boolean array defining which frames are required during runtime. */
  FrameRefVector default_frames_required_;   /**< Boolean array defining which frames are required by default during runtime. */
  FrameRefVector frame_exists_;              /**< Boolean array defining which frames currently exist during a given iteration. */

#if OPENCV_CUDA
  cv::cuda::GpuMat hd_frame_cuda_;           /**< The unaltered image of the current frame uploaded to the GPU. */
  cv::cuda::GpuMat sd_frame_cuda_;           /**< Resized common::System::hd_frame_cuda image by common::System::resize_scale_. */
  cv::cuda::GpuMat mono_frame_cuda_;         /**< Converted common::System::sd_frame_cuda_ image as greyscale image using CV_RGB2GRAY. */
  cv::cuda::GpuMat undist_frame_cuda_;       /**< Converted common::System::sd_frame_cuda_ image as undistorted image using cv::undistort. */
  cv::cuda::GpuMat hsv_frame_cuda_;          /**< Converted common::System::sd_frame_cuda_ image as HSV image using CV_BGR2HSV. */
  cv::cuda::GpuMat undistort_map_x_;         /**< Distortion map for x pixel coordinates used for producing undistorted CUDA frame. */
  cv::cuda::GpuMat undistort_map_y_;         /**< Distortion map for y pixel coordinates used for producing undistorted CUDA frame. */
  CUDAFrameRefVector cuda_frames_required_;  /**< Boolean array defining which CUDA frames are required during runtime. */
  CUDAFrameRefVector default_cuda_frames_required_;  /**< Boolean array defining which CUDA frames are required by default during runtime. */
  CUDAFrameRefVector cuda_frame_exists_;     /**< Boolean array defining which CUDA frames currently exist during a given iteration. */
#endif

  double resize_scale_;                      /**< common::System::hd_frame_ is resized by this parameter to make common::System::sd_frame_. */ 
  bool cam_info_received_;                   /**< Flag used to indicate if camera information has been recieved. The camera information is used
                                                  to create common::System::hd_camera_matrix_ and common::System::dist_coeff_*/
  
  bool tuning_;                              /**< Flag used to indicate if the plugins should generate images to help with debuggin or tunning. */

  int message_output_period_;                /**< Print a ROS message at most once per "text_output_period_" */
  
  static std::string picture_file_path_;     /**< File Path where the pictures will be saved. */
  static bool picture_file_path_set_;        /**< If True, the picture file path has been set but no garuntee that its a valid file path. */

  double current_time_ = 0;                  /**< The current time in seconds provided by the video header files. */
  double prev_time_ = 0;                     /**< The time stamp associated with the previous image from which d_prev_matched_ were extracted. When the first image is received,
                                                  prev_time_ is set to current time since there is no previous image. */

  // RRANSAC visualization info
  rransac::DrawInfo rransac_draw_info_;      /**< The visualization info for RRANSAC */
  std::string rransac_video_file_name_;      /**< The filename of the video. This must be an absolute path. Ex: /home/user/vmtt/video.mp4. */
  double rransac_fps_;                       /**< The video's fps for the RRANSAC visualization. */
  bool rransac_visualize_data_;              /**< If true, the visualization of RRANSAC wil be shown. */
  int rransac_vis_image_width_;              /**< The width of the image that will display the rransac visualization. */
  int rransac_vis_image_height_;             /**< The height of the image that will display the rransac visualization. */


#if TRACKING_SE2
  rransac::VisualizationHost<RR_Model, rransac::DrawMeasR2SE2PosPolicy, rransac::DrawTrackPolicySE2> rransac_viz_;
#else // R2
  rransac::VisualizationHost<RR_Model, rransac::DrawMeasR2SE2PosPolicy, rransac::DrawTrackPolicyR2> rransac_viz_;
#endif


  private:

};


}

