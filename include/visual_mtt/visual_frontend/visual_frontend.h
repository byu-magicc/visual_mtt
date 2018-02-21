#pragma once

// libraries
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <rransac/tracker.h>

// dynamic reconfig
#include "visual_mtt/rransacConfig.h"
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

#include "source_manager/feature_motion.h"

namespace visual_frontend {

  class VisualFrontend
  {
  public:
    VisualFrontend();


  private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::CameraSubscriber sub_video;
    // ros::Subscriber sub_tracks;
    // ros::Publisher  pub_scan;
    // ros::Publisher  pub_stats;

    ros::Publisher  pub_tracks;
    image_transport::Publisher pub_tracks_video;

    // dynamic reconfigure server and service client for R-RANSAC params
    dynamic_reconfigure::Server<visual_mtt::visual_frontendConfig> server_;
    std::unique_ptr<dynamic_reconfigure::Server<visual_mtt::rransacConfig>> rransac_server_;
    ros::ServiceServer srv_recognize_track_;

    // algorithm managers
    FeatureManager     feature_manager_;
    HomographyManager  homography_manager_;
    SourceManager      source_manager_;
    RecognitionManager recognition_manager_;

    // Recursive-RANSAC Tracker
    rransac::Tracker tracker_;
    rransac::core::Parameters params_;
    std::vector<cv::Point2f> corner_{{0,0}};
    std::vector<cv::Scalar> colors_;

    int frame_ = 0;

    // R-RANSAC iteration counter
    int iter_num_ = 0;
    double pub_scale_ = 1;
    double text_scale_ = 1;

    // camera parameters
    ros::Time timestamp_frame_;
    cv::Mat camera_matrix_, camera_matrix_scaled_;
    cv::Mat dist_coeff_;
    bool info_received_ = false;

    visual_mtt::TracksPtr tracks_;

    // only process every `frame_stride_` frames
    unsigned int frame_stride_;

    // resize scale
    double resize_scale_;

    // frames
    cv::Mat hd_frame_;
    cv::Mat sd_frame_;
    cv::Size hd_res_;
    cv::Size sd_res_;

    // subscription and dynamic reconfigure callbacks
    void callback_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);
    void callback_tracks(const visual_mtt::TracksPtr& data);
    void callback_reconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level);
    void callback_reconfigure_rransac(visual_mtt::rransacConfig& config, uint32_t level);
    bool callback_srv_recognize_track(visual_mtt::RecognizeTrack::Request &req, visual_mtt::RecognizeTrack::Response &res);

    // data management
    void set_parameters(visual_mtt::visual_frontendConfig& config);

    // Take R-RANSAC Tracker output and publish to ROS (i.e., Good Models)
    void publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks);

    // Draw tracks over original frame
    void draw_tracks(const std::vector<rransac::core::ModelPtr>& tracks);
  };

}
