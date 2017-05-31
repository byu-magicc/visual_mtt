#pragma once

#include <rransac/tracker.h>
#include <rransac/access_type.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h" // temporary include for temporary message type (for compilation)

namespace visual_mtt {

  class RRANSAC
  {
  public:
    RRANSAC();
    //~RRANSAC();

    void callback(const std_msgs::Float32); // temporary message type, in future, use custom homography+measurements message

  private:
    rransac::core::Parameters params_;
    rransac::Tracker tracker_;

    // ROS
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

  };

}
