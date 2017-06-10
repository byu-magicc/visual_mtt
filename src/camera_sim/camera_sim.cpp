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

// This only shows the syntax for dual-message publishing and image transport.
// Eventually read in video rather than subscribe, then replace video_player.py

// Near-term list:
// TODO: cpp header file
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

  // image data
  cv::Mat frame_;

  // camera manager class
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_manager_;
};

// ----------------------------------------------------------------------------

CameraSim::CameraSim()
{
  // private node handle for params and keeping image topics organized
  ros::NodeHandle nh("~");

  // get parameters from param server
  std::string camera_name, camera_info_path;
  nh.param<std::string>("camera_name",      camera_name,      "");
  nh.param<std::string>("camera_info_path", camera_info_path, "");

  // configure the camera manager class, this gets info from the camera .yaml
  // docs.ros.org/kinetic/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html
  camera_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name, camera_info_path));

  // ROS communication
  image_transport::ImageTransport it(nh); // use private node handle
  sub_ = nh_.subscribe("input", 1, &CameraSim::callback,  this);
  pub_ = it.advertiseCamera("image_raw", 1);
}

// ----------------------------------------------------------------------------

void CameraSim::callback(const sensor_msgs::ImageConstPtr& data)
{
  // convert image to OpenCV because that's how it will be in the future
  // when it's read from .mp4 file, eventually replacing the python script
  frame_ = cv_bridge::toCvCopy(data, "bgr8")->image;

  // sanity check: compare dimensions of frame to .yaml (on first frame?)
  // stuff, warning

  // convert to image transport
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
  msg->header = data->header;

  // get the camera info
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_manager_->getCameraInfo()));
  ci->header.stamp = data->header.stamp;

  // publish frame and camera info together (can only publish <msg>Ptr types)
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