#pragma once

#include <ros/ros.h>

#include <rransac/tracker.h>
#include <rransac/access_type.h>

#include "rransac/accessors.h"

#include "visual_mtt2/RRANSACScan.h"
#include "visual_mtt2/Source.h"
#include "visual_mtt2/Measurement.h"
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)


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

    // ROS subscriber callback. Each callback a new measurement
    // scan is received and the R-RANSAC Tracker is run.
    void callback(const visual_mtt2::RRANSACScanPtr& rransac_scan);

    // Take R-RANSAC Tracker output and publish to ROS (i.e., Good Models)
    void publish_tracks(std::vector<rransac::core::ModelPtr> tracks);

  };

}
