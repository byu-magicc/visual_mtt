// libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

// messages
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"

// This only shows the syntax for dual-message publishing and image transport.
// Eventually read in video rather than subscribe, then replace video_player.py

// Near-term list:
// TODO: header file
// TODO: in usb_cam, the camera .yaml file path is put on the param server,
// then node pulls in the yaml values on its own.
// TODO: separate into node.cpp and camera_sim.cpp to match other nodes?
// TODO: add namespace

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
  image_transport::CameraPublisher pub_;

  // data
  cv::Mat frame_;
};

// ----------------------------------------------------------------------------

CameraSim::CameraSim()
{
  // create a private node handle for use with param server
  ros::NodeHandle nh("~"); // not used yet

  // get parameters from param server that are not dynamically reconfigurable
  // stuff

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_ = nh_.subscribe("video", 1, &CameraSim::callback,  this);
  pub_ = it.advertiseCamera("camera/image", 1);
}

// ----------------------------------------------------------------------------

void CameraSim::callback(const sensor_msgs::ImageConstPtr& data)
{
  // convert image to OpenCV because that's how it will be in the future
  // when it's read from .mp4 file.
  frame_ = cv_bridge::toCvCopy(data, "bgr8")->image;

  // convert to image transport
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();

  // create empty camera info for testing
  sensor_msgs::CameraInfoPtr ci; // arguments in usb_cam: (new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()))

  // publish two messages together
  pub_.publish(msg, ci);
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