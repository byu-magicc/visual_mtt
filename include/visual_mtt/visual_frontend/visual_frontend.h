#pragma once

// libraries
#include <iostream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

// dynamic reconfig
#include "visual_mtt/visual_frontendConfig.h"

// messages
#include "visual_mtt/Tracks.h"
#include "visual_mtt/RRANSACScan.h"
#include "visual_mtt/Stats.h"
#include "visual_mtt/RRANSACParams.h"
#include "visual_mtt/RecognizeTrack.h"
#include "sensor_msgs/Image.h"

// key algorithm members
#include "feature_manager/feature_manager.h"
#include "homography_manager/homography_manager.h"
#include "source_manager/source_manager.h"
#include "recognition_manager/recognition_manager.h"

#include "source_manager/feature_outliers.h"

namespace visual_frontend {

  class VisualFrontend
  {
  public:
    VisualFrontend();


  private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::CameraSubscriber sub_video;
    ros::Subscriber sub_tracks;
    ros::Publisher  pub_scan;
    ros::Publisher  pub_stats;

    // dynamic reconfigure server and service client for R-RANSAC params
    dynamic_reconfigure::Server<visual_mtt::visual_frontendConfig> server_;
    ros::ServiceClient srv_params_;
    ros::ServiceServer srv_recognize_track_;
    visual_mtt::RRANSACParams srv_saved_;
    bool srv_resend_;

    // algorithm managers
    FeatureManager     feature_manager_;
    HomographyManager  homography_manager_;
    SourceManager      source_manager_;
    RecognitionManager recognition_manager_;

    // camera parameters
    ros::Time timestamp_frame_;
    cv::Mat camera_matrix_, camera_matrix_scaled_;
    cv::Mat dist_coeff_;
    bool info_received_ = false;

    visual_mtt::TracksPtr tracks_;

    // only process every `frame_stride_` frames
    unsigned int frame_stride_;

    // downsize scale
    double downsize_scale_;

    // frames
    cv::Mat hd_frame_;
    cv::Mat sd_frame_;
    cv::Size hd_res_;
    cv::Size sd_res_;

    // subscription and dynamic reconfigure callbacks
    void callback_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);
    void callback_tracks(const visual_mtt::TracksPtr& data);
    void callback_reconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level);
    bool callback_srv_recognize_track(visual_mtt::RecognizeTrack::Request &req, visual_mtt::RecognizeTrack::Response &res);

    // data management
    void set_parameters(visual_mtt::visual_frontendConfig& config);
    void srv_set_params(visual_mtt::visual_frontendConfig& config);
  };

}
