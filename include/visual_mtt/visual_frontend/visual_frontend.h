#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)
#include "sensor_msgs/Image.h" // needed for subscription to video message
#include "homography_calculator.h"
#include "feature_manager.h"
#include "source_features.h"

namespace visual_mtt {

	class VisualFrontend
	{
	public:
		VisualFrontend();
		//~VisualFrontend();

    void callback_video(const sensor_msgs::ImageConstPtr&);
    void callback_imu(const std_msgs::Float32); // temporary message type
    void callback_tracks(const std_msgs::Float32); // temporary message type

    void add_frame(cv::Mat&, cv::Mat&); // second argument: uMat

    void generate_measurements();

    cv::Mat hd_frame_in;
    cv::Mat sd_frame_in = cv::Mat(480, 640, CV_8UC3);                           // TODO: parameterize dimensions

    cv::Mat hd_frame; // uMat
    cv::Mat sd_frame; // uMat

    int test_val = 24;

  private:
    // ROS
		ros::NodeHandle nh;
		ros::Subscriber sub_video;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_tracks;
		ros::Publisher  pub;

    // key algorithm blocks
		HomographyCalculator homography_calculator_;
    FeatureManager feature_manager_;

    // measurement sources
    std::vector<SourceMeasurement> sources_;

	};

}