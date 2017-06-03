#pragma once

#include <ros/ros.h>

#include <rransac/tracker.h>
#include <rransac/access_type.h>

#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)

#include "rransac/opencv_accessors.h"

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
    void callback(const std_msgs::Float32);

    // Take R-RANSAC Tracker output and publish to ROS (i.e., Good Models)
    void publish_tracks(std::vector<rransac::core::ModelPtr> tracks);

  };

}
