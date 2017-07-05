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
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation of imu callback)
#include "sensor_msgs/Image.h" // needed for subscription to video message

// key algorithm members
#include "feature_manager/feature_manager.h"
#include "homography_manager/homography_manager.h"
#include "source_features.h"

namespace visual_frontend {

  class VisualFrontend
  {
  public:
    VisualFrontend();

    // subscription and dynamic reconfigure callbacks
    void callback_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);
    void callback_imu(const std_msgs::Float32 data);    // temporary message type
    void callback_tracks(const visual_mtt::TracksPtr& data);
    void callback_reconfigure(visual_mtt::visual_frontendConfig& config, uint32_t level);

    // data management and execution
    void set_parameters(visual_mtt::visual_frontendConfig& config);
    void generate_measurements();

    // cv::Mat frame_in;
    // cv::Mat sd_frame_in;
    cv::Mat hd_frame;
    cv::Mat sd_frame;

    visual_mtt::TracksPtr tracks_;

  private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::CameraSubscriber sub_video;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_tracks;
    ros::Publisher  pub_scan;
    ros::Publisher  pub_stats;

    // dynamic reconfigure server
    dynamic_reconfigure::Server<visual_mtt::visual_frontendConfig> server_;

    // algorithm managers
    FeatureManager    feature_manager_;
    HomographyManager homography_manager_;

    // camera parameters
    ros::Time timestamp_frame_;
    cv::Mat camera_matrix_, camera_matrix_scaled_;
    cv::Mat dist_coeff_;
    bool info_received_ = false;

    // measurement sources
    std::vector<std::shared_ptr<Source>> sources_;

    // tuning mode
    bool tuning_;

    // Only plot process every `frame_stride_` frames
    unsigned int frame_stride_;

    // downsize scale
    double downsize_scale_;
  };

}