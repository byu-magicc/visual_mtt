#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rransac/tracker.h>
#include <rransac/access_type.h>

#include "rransac/accessors.h"

#include "visual_mtt/rransacConfig.h"
#include "visual_mtt/RRANSACScan.h"
#include "visual_mtt/Source.h"
#include "visual_mtt/Measurement.h"
#include "visual_mtt/Tracks.h"
#include "visual_mtt/Track.h"
#include "visual_mtt/Stats.h"
#include "visual_mtt/RRANSACParams.h"
#include "sensor_msgs/Image.h"

#include <iostream>
#include <algorithm>

namespace rransac {

  class RRANSAC
  {
  public:
    RRANSAC();

  private:
    rransac::core::Parameters params_;
    rransac::Tracker tracker_;

    // ROS pub/sub
    ros::NodeHandle nh;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_stats;
    ros::Publisher pub;
    image_transport::CameraSubscriber sub_video;
    image_transport::Publisher pub_tracks_video;

    // Saved frame and scan headers, received at each callback
    std_msgs::Header header_frame_last_;
    std_msgs::Header header_frame_;
    int frame_seq_;

    // Low-pass filter for fps and utilization
    visual_mtt::Utilization util_;   // store utilization values
    double t_rransac_       = 0;
    double fps_             = 30;
    int    frame_stride_    = 1;
    double alpha_           = 0.003; // fps filter: large time constant ~10s
    double time_constant_   = 1.0;   // utilization filter: chose time constant

    // For visualization
    cv::Mat frame_;
    double pub_scale_;
    std::vector<cv::Scalar> colors_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeff_;
    bool info_received_ = false;
    std::vector<cv::Point2f> corner_;

    // dynamic reconfigure and service server
    dynamic_reconfigure::Server<visual_mtt::rransacConfig> server_;
    ros::ServiceServer srv_params_;

    // Dynamic reconfigure callback
    void callback_reconfigure(visual_mtt::rransacConfig& config, uint32_t level);
    bool callback_srv_params(visual_mtt::RRANSACParams::Request &req, visual_mtt::RRANSACParams::Response &res);

    // ROS subscriber callback. Each callback a new measurement
    // scan is received and the R-RANSAC Tracker is run.
    void callback_scan(const visual_mtt::RRANSACScanPtr& rransac_scan);

    // ROS subscriber callback. Each callback a new set of stats
    // stats are received and local members are updated.
    void callback_stats(const visual_mtt::Stats& data);

    // ROS subscriber callback. Each callback a new frame
    // frame is saved for drawing the tracking results.
    void callback_video(const sensor_msgs::ImageConstPtr& frame, const sensor_msgs::CameraInfoConstPtr& cinfo);

    // Take R-RANSAC Tracker output and publish to ROS (i.e., Good Models)
    void publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks);

    // Draw tracks over original frame
    void draw_tracks(const std::vector<rransac::core::ModelPtr>& tracks);

    // Saved source information
    visual_mtt::RRANSACParams::Request req_last_;
  };

}