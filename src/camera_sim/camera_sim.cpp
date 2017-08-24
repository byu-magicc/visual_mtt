#include "camera_sim/camera_sim.h"

namespace camera_sim {

CameraSim::CameraSim()
{
  // private node handle for params and keeping image topics organized
  ros::NodeHandle nh_private("~");

  // get parameters from param server
  std::string camera_name, camera_info_url;
  nh_private.param<std::string>("camera_name",     camera_name,     "");
  nh_private.param<std::string>("camera_info_url", camera_info_url, "");
  nh_private.param<std::string>("video_path",      video_path_,     "");
  nh_private.param<double>     ("fps",             fps_,            30);

  video_file_only_ = false;
  if (video_path_ != "")
    video_file_only_ = true;

  // if the default camera_sim is being used, parameters are being guessed
  if (camera_name=="camera_sim")
  {
    parameters_guessed_ = true;
    ROS_WARN("camera sim: intrinsic camera parameters will be guessed");
  }

  // configure the camera manager class, this gets info from the camera .yaml
  // docs.ros.org/kinetic/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html
  camera_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name, camera_info_url));

  // ROS communication
  // if the video source is a rosbag, subscription is needed
  // if the video source is a standard file, there is no need to subscribe
  image_transport::ImageTransport it(nh_private);
  pub_ = it.advertiseCamera("image_raw", 1);
  if (!video_file_only_)
    sub_ = nh_.subscribe("input", 1, &CameraSim::callback,  this);
}

// ----------------------------------------------------------------------------

void CameraSim::callback(const sensor_msgs::ImageConstPtr& data)
{
  // convert image to OpenCV
  frame_ = cv_bridge::toCvCopy(data, "bgr8")->image;

  // convert to image transport
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
  msg->header = data->header;

  // get the camera info
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_manager_->getCameraInfo()));
  ci->header.stamp = data->header.stamp;

  // update the height, width, and principal point if parameters are guessed
  if (parameters_guessed_)
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

// ----------------------------------------------------------------------------

void CameraSim::play_video()
{
  // only run this video loop if the video source is a standard file
  if (!video_file_only_)
    return;

  cv::VideoCapture source(video_path_);

  if (!source.isOpened())
    ROS_ERROR("Invalid video file path");

  // time regulation
  ros::Time tic_fps = ros::Time::now();
  ros::Time tic_publish;
  ros::Duration spf = ros::Duration(1/fps_);  // seconds per frame
  ros::Duration t_publish = ros::Duration(0); // estimate of publish cost
  ros::Duration t_publish_i;                  // measured cost (one iteration)
  ros::Duration t_elapsed;                    // time since last frame
  ros::Duration t_remaining;                  // time until next frame

  // alpha for lpf of t_publish (based on time constant of 3s)
  double alpha = spf.toSec()/(3 + spf.toSec());

  // latch logic for slow playback warning
  bool warning_latched = false;
  uint32_t latch_counter = 0;

  while (ros::ok()) {
    // get the next frame; loop if the end of the video is reached
    source >> frame_;
    if(frame_.empty())
    {
      ROS_INFO("Looping video");
      source.release();
      source.open(video_path_);
      source >> frame_;
    }

    // convert frame to image transport message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();

    // get the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_manager_->getCameraInfo()));

    // update the height, width, and principal point if parameters are guessed
    if (parameters_guessed_)
    {
      ci->height = frame_.rows;
      ci->width = frame_.cols;
      // ci->K[0] = 700;                // fx (keep original guess)
      // ci->K[4] = 700;                // fy (keep original guess)
      ci->K[2] = (double)frame_.cols/2; // cx
      ci->K[5] = (double)frame_.rows/2; // cy
    }

    t_elapsed = ros::Time::now() - tic_fps;
    t_remaining = spf - t_elapsed - t_publish;

    // less-than-real-time logic and warning latch logic
    if (t_remaining.toSec()<0)
    {
      if (!warning_latched) latch_counter++;

      // if the latch threshold is met, latch and fire warning
      if (latch_counter>=10 && !warning_latched)
      {
        ROS_WARN("Video cannot be published at the requested fps");
        warning_latched = true;
      }
    }
    else
    {
      // reset latch counter and wait for the remaining time
      latch_counter = 0;
      t_remaining.sleep();
    }

    // the frame is not available to other nodes until the end of the
    // publication command, so we estimate the time cost of publishing (which
    // is pretty consistent) in order to anticipate and offset it.
    tic_publish = ros::Time::now();

    // publish frame and camera info together (can only publish <msg>Ptr types)
    msg->header.stamp = ros::Time::now();
    ci->header.stamp = msg->header.stamp;
    pub_.publish(msg, ci);

    // update the "time of last frame" tic for the next iteration
    tic_fps = ros::Time::now();

    // calculate the time cost of publishing this iteration
    t_publish_i = ros::Time::now()-tic_publish;

    // lpf the time cost of publishing
    if (t_publish.toSec()==0)
      t_publish = t_publish_i; // first iteration
    else
      t_publish = t_publish_i*alpha + t_publish*(1-alpha);
  }
}

}