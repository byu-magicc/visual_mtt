#pragma once

// libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// dynamic reconfig
#include "visual_mtt2/visual_frontendConfig.h"

// messages
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)
#include "sensor_msgs/Image.h" // needed for subscription to video message

// key algorithm members
#include "feature_manager.h"
#include "homography_calculator.h"
#include "source_background.h"
#include "source_features.h"

namespace visual_mtt {

  class VisualFrontend
  {
  public:
    VisualFrontend();
    //~VisualFrontend();

    // subscription and dynamic reconfigure callbacks
    void callback_video(const sensor_msgs::ImageConstPtr& data);
    void callback_imu(const std_msgs::Float32 data);    // temporary message type
    void callback_tracks(const std_msgs::Float32 data); // temporary message type
    void callback_reconfigure(visual_mtt2::visual_frontendConfig& config, uint32_t level);

    // data management and execution
    void set_parameters(visual_mtt2::visual_frontendConfig& config);
    void add_frame(cv::Mat& newMat, cv::Mat& memberMat); // second argument: uMat
    void generate_measurements();

    // after v1.0, there may be collections of recent frames and associated
    // timestamps, "add_frame" will sort of become a manager of these histories
    // (and provide CPU/GPU support of course)

    cv::Mat calibration_; // not used yet
    cv::Mat distortion_;  // not used yet

    cv::Mat hd_frame_in;
    cv::Mat sd_frame_in = cv::Mat(480, 640, CV_8UC3);                           // TODO: parameterize dimensions (maybe add logic for aspect ratio)
    cv::Mat hd_frame; // uMat
    cv::Mat sd_frame; // uMat

    ros::Time frame_timestamp_;

  private:
    // ROS
    ros::NodeHandle nh;
    ros::Subscriber sub_video;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_tracks;
    ros::Publisher  pub;

    // dynamic reconfigure server
    dynamic_reconfigure::Server<visual_mtt2::visual_frontendConfig> server_;

    // key algorithm members
    std::shared_ptr<FeatureManager>       feature_manager_;
    std::shared_ptr<HomographyCalculator> homography_calculator_;

    // measurement sources
    std::vector<std::shared_ptr<Source>> sources_;

  };

}