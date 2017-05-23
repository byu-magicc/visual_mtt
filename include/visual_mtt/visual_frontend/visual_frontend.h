#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)
#include "sensor_msgs/Image.h" // needed for subscription to video message

namespace visual_mtt {

	class VisualFrontend
	{
	public:
		VisualFrontend();
		//~VisualFrontend();

    void callback_video(const sensor_msgs::ImageConstPtr&);
    void callback_imu(const std_msgs::Float32); // temporary message type
    void callback_tracks(const std_msgs::Float32); // temporary message type

  private:
    // ROS
		ros::NodeHandle nh;
		ros::Subscriber sub_video;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_tracks;
		ros::Publisher pub;

	};

}