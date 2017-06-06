#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <rransac/tracker.h>
#include <rransac/access_type.h>

#include "rransac/accessors.h"

#include "visual_mtt2/rransacConfig.h"
#include "visual_mtt2/RRANSACScan.h"
#include "visual_mtt2/Source.h"
#include "visual_mtt2/Measurement.h"
#include "visual_mtt2/Tracks.h"
#include "visual_mtt2/Track.h"

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
    ros::Subscriber sub;
    ros::Publisher pub;

    // dynamic reconfigure server
    dynamic_reconfigure::Server<visual_mtt2::rransacConfig> server_;

    // Dynamic reconfigure callback
    void callback_reconfigure(visual_mtt2::rransacConfig& config, uint32_t level);

    // ROS subscriber callback. Each callback a new measurement
    // scan is received and the R-RANSAC Tracker is run.
    void callback(const visual_mtt2::RRANSACScanPtr& rransac_scan);

    // Take R-RANSAC Tracker output and publish to ROS (i.e., Good Models)
    void publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks);

    // Saved frame header, received at each callback
    std_msgs::Header header_frame_;

  };

}
