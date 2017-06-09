// libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/console.h>

// messages
#include "sensor_msgs/Image.h" // needed for subscription to video message

// This only shows the syntax for dual-message publishing and image transport.
// Eventually read in video rather than subscribe, then replace video_player.py

// TODO: header file
// TODO: in usb_cam, the camera .yaml file path is put on the param server,
// then node pulls in the yaml values on its own.

class CameraSim
{
public:
  CameraSim();

  // subscription
  void callback(const sensor_msgs::ImageConstPtr& data);

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_;
};

// ----------------------------------------------------------------------------

CameraSim::CameraSim()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh("~");

  // get parameters from param server that are not dynamically reconfigurable
  // stuff

  // ROS communication
  sub_ = nh_.subscribe("video", 1, &CameraSim::callback,  this);
  // pub_ = nh_.advertise<>("measurements", 1);
}

// ----------------------------------------------------------------------------

void CameraSim::callback(const sensor_msgs::ImageConstPtr& data)
{
  ROS_INFO("what up");

  // convert image to OpenCV because that's how it will be in the future
  // when it's read from .mp4 file.

}

// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{
  // start node
  ros::init(argc, argv, "camera_sim");

  // instantiate the CameraSim class
  CameraSim camera_sim;

  ros::spin();
  return 0;
}