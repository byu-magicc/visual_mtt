// libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/console.h>

// messages
#include "sensor_msgs/Image.h" // needed for subscription to video message


// This only shows the syntax for dual-message publishing and image transport.
// Eventually, make a class, read in images, and replace video_player.py

// TODO: header file
// TODO: in usb_cam, the camera .yaml file path is put on the param server,
// then node pulls in the yaml values on its own.


void callback_video(const sensor_msgs::ImageConstPtr& data)
{
  ROS_INFO("what up");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "camera_sim");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("video", 1, callback_video);

  ros::spin();

  return 0;
}