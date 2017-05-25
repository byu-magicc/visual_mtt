#include "visual_frontend/visual_frontend.h"

namespace visual_mtt {

VisualFrontend::VisualFrontend()
{
  // receive parameters from launchfile
  // bool x, y, z;
	// nh.param<bool>("show_x", x, false);
	// nh.param<bool>("show_y", y, false);
	// nh.param<bool>("show_z", z, false);
  // the original code used "nh_private_": "ros::NodeHandle nh_private_("~");"

  std::cout << "initialized VisualFrontend object inside node" << std::endl; // temporary

  // ROS stuff
  sub_video  = nh.subscribe("video",  1, &VisualFrontend::callback_video,  this);
  sub_imu    = nh.subscribe("imu",    1, &VisualFrontend::callback_imu,    this);
  sub_tracks = nh.subscribe("tracks", 1, &VisualFrontend::callback_tracks, this);
	pub        = nh.advertise<std_msgs::Float32>("measurements_and_homographies", 1); // temporary dummy std_msgs for compilation

  SourceFeatures test_instantiation;
  test_instantiation.add_handle(this);
  test_instantiation.retrieve_info();

}


void VisualFrontend::callback_video(const sensor_msgs::ImageConstPtr& data)
{
  // future work TODO:
  // decimation logic (Nth frame)
  // resize frame to lower resolution (keep both)
    // a short history is needed for the low res for the sliding
    // a short history is needed for the high res so the track recognition can
    // locate the high-res frame associated with the track it's subscribing to


  // generate timestamp
  ros::Time frame_timestamp = ros::Time::now(); // make class member

  // convert message data into OpenCV type cv::Mat
	hd_frame_in = cv_bridge::toCvCopy(data, "bgr8")->image;

  // generate standard definition image
  cv::resize(hd_frame_in, sd_frame_in, sd_frame_in.size(), 0, 0, cv::INTER_LINEAR);

  // add frames to recent history
  add_frame(hd_frame_in, hd_frame);
  add_frame(sd_frame_in, sd_frame);

  // manage features (could be LK, NN, Brute Force)
  feature_manager_.find_correspondences();                                      // (make smart pointer)

  // consider if IMU is ignored (param from launchfile)
  // if IMU     ignored, call homography_generator (feature correspondences)
  // if IMU not ignored, call the homography_filter (filter update)
  homography_calculator_.calculate_homography();                                // (make smart pointer)

  // call measurement sources execution
      // (use updated recent images)
      // (use already-generated feature correpsondences)
      // (use already-generated homography)
      // (use updated recent track data)
  generate_measurements();

  // publish measurements and homography


  // display frames
  cv::imshow("hd image", hd_frame);
  cv::imshow("sd image", sd_frame);

  // get the input from the keyboard
	char keyboard = cv::waitKey(10);
	if(keyboard == 'q')
		ros::shutdown();
}

void VisualFrontend::callback_imu(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
  // we expect to be getting data for this at >500 Hz
  //
  // for in-frame tracking, we'll need 2 homographies:
  // one between the sliding frames for background subtraction
  // one between the two recent frames for updating history and estimates
  // if the homography filter operates on a manifold, maybe it can help?
  // NOTE: need better understanding of homography filter to answer this.
  // ---------------------------------------------------

  // if IMU not ignored (from launchfile), call homography filter (IMU input)


}

void VisualFrontend::callback_tracks(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
	// save most recent track information in class (for use in measurement
  // sources such as direct methods)

  // call track_recognition bank (will use newest information and the high-res
  // video associated with the most recent update to maintain id descriptors.)

}



void VisualFrontend::add_frame(cv::Mat& newMat, cv::Mat& memberMat) // second argument: uMat
{

  // why does this need its own method?
  // see https://github.com/jdmillard/opencv-cuda
  memberMat = newMat;

}


void VisualFrontend::generate_measurements()
{

  for (int i=0; i<sources_.size(); i++)
  {
    sources_[i].generate_measurements();
  }
  // pass "this" class' reference in? so the sources can indiscriminately use
  // whatever valuable information they need? and maintain polymorphism.
  // (frame history, homography, features/feature velocities, recent tracks)

  // see source_measurement.h for question about measurement structure

}




}
