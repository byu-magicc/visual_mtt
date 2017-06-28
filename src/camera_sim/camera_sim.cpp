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

namespace camera_sim {

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

    bool guessed_;

    // camera manager class
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_manager_;
  };

// ----------------------------------------------------------------------------

CameraSim::CameraSim()
{
  // private node handle for params and keeping image topics organized
  ros::NodeHandle nh_private("~");

  // get parameters from param server
  std::string camera_name, camera_info_url;
  nh_private.param<std::string>("camera_name",     camera_name,     "");
  nh_private.param<std::string>("camera_info_url", camera_info_url, "");

  // if the default camera_sim is being used, parameters are being guessed
  if (camera_name=="camera_sim")
  {
    guessed_ = true;
    ROS_WARN("camera sim: intrinsic camera parameters will be guessed");
  }

  // configure the camera manager class, this gets info from the camera .yaml
  // docs.ros.org/kinetic/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html
  camera_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name, camera_info_url));

  // ROS communication
  image_transport::ImageTransport it(nh_private); // use private node handle
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

  // update the height, width, and principal point if parameters are guessed
  if (guessed_)
  {
    ci->height = frame_.rows;
    ci->width = frame_.cols;
    // ci->K[0] = 700;                // fx (keep original guess)
    // ci->K[4] = 700;                // fy (keep original guess)
    ci->K[2] = (double)frame_.cols/2; // cx
    ci->K[5] = (double)frame_.rows/2; // cy
  }

  // Make sure there is an actual timestamp
  if (msg->header.stamp.sec == 0) {
    msg->header.stamp = ros::Time::now();
    ci->header.stamp = msg->header.stamp;
  }

  // publish frame and camera info together (can only publish <msg>Ptr types)
  pub_.publish(msg, ci);
}

}

// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{
  // start node
  ros::init(argc, argv, "camera_sim");

  // instantiate the CameraSim class
  camera_sim::CameraSim camera_sim;

  ros::spin();
  return 0;
}
