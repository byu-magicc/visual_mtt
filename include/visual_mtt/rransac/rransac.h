#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rransac/tracker.h>
#include <rransac/access_type.h>

#include "rransac/accessors.h"

#include "visual_mtt2/rransacConfig.h"
#include "visual_mtt2/RRANSACScan.h"
#include "visual_mtt2/Source.h"
#include "visual_mtt2/Measurement.h"
#include "visual_mtt2/Tracks.h"
#include "visual_mtt2/Track.h"
#include "visual_mtt2/Stats.h"
#include "sensor_msgs/Image.h"

#include <iostream>
#include <algorithm>

namespace visual_mtt {

  class RRANSAC
  {
  public:
    RRANSAC();

  private:
    rransac::core::Parameters params_;
    rransac::Tracker tracker_;

    // ROS
    ros::NodeHandle nh;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_stats;
    image_transport::Subscriber sub_video;
    ros::Publisher pub;

    // Saved frame and scan headers, received at each callback
    std_msgs::Header header_frame_last_;
    std_msgs::Header header_frame_;
    int frame_seq_;

    // Low-pass filter for fps and utilization
    double fps_           = 30;
    int    frame_stride_  = 1;
    double utilization_   = 0.75;
    double alpha_         = 0.003; // fps filter: large time constant ~10s
    double time_constant_ = 1.5;   // utilization filter: chose time constant

    // For visualization
    cv::Mat frame_;
    std::vector<cv::Scalar> colors_;
    bool show_tracks_;

    // dynamic reconfigure server
    dynamic_reconfigure::Server<visual_mtt2::rransacConfig> server_;

    // Dynamic reconfigure callback
    void callback_reconfigure(visual_mtt2::rransacConfig& config, uint32_t level);

    // ROS subscriber callback. Each callback a new measurement
    // scan is received and the R-RANSAC Tracker is run.
    void callback_scan(const visual_mtt2::RRANSACScanPtr& rransac_scan);

    // ROS subscriber callback. Each callback a new set of stats
    // stats are received and local members are updated.
    void callback_stats(const visual_mtt2::Stats& data);

    // ROS subscriber callback. Each callback a new frame
    // frame is saved for drawing the tracking results.
    void callback_video(const sensor_msgs::ImageConstPtr& frame);

    // Take R-RANSAC Tracker output and publish to ROS (i.e., Good Models)
    void publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks);

    // Draw tracks over original frame
    void draw_tracks(const std::vector<rransac::core::ModelPtr>& tracks);
  };

}