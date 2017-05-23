#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)

namespace visual_mtt {

	class VisualFrontend
	{
	public:
		VisualFrontend();
		//~VisualFrontend();

    void callback_video(const std_msgs::Float32); // temporary message type
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