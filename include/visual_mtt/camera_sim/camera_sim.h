#pragma once

// libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>

// messages
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"

namespace camera_sim {

  class CameraSim
  {
  public:
    CameraSim();

    // subscription
    void callback(const sensor_msgs::ImageConstPtr& data);

    // video player
    void play_video();

  private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    image_transport::CameraPublisher pub_;

    // image data
    cv::Mat frame_;

    // video properties
    bool parameters_guessed_;
    bool video_file_only_;
    std::string video_path_;
    double fps_;

    // camera manager class
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_manager_;
  };

}