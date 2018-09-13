#pragma once

// std
#include <fstream>
#include <string>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <visual_mtt/Tracks.h>


namespace visual_mtt_benchmark {

/** \class Benchmark
* \details Collects all of the unilization data provided by visual_frontend::VisualFrontend
* and generates statistics which are written to the a file indicated by
* Benchmark::file_path_. This file can then be used by the matlab script
* benchmark/matlab/benchmark.m to generate plots.
*
* \author Mark Petersen
*/

  class Benchmark {

  public:


  /**
  * \details Setup ROS communication and retrieves the file path name
  * to save the data. This file path is saved to Benchmark::file_path_.
  */
    Benchmark();

  /**
  * \details Calls Benchmark::WriteCsvFile() to generate and write all of the
  * utilization statistics. 
  */
    ~Benchmark();

  private:

    /**
    * \details Genererates and writes all of the utilization statistics to 
    * a CSV file that can be used by a matlab script to view
    * the statistics in a plot. 
    */
    void WriteCsvFile();

    /**
    * \details Stores the incoming message data to Benchmark::tracks_data_.
    * This data will be used later to calculate utilization statistics. 
    */
    void CallbackTracks(const visual_mtt::Tracks& msg);


    // ROS
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_tracks_;

    std::string file_path_; /**< The file path to the CSV file to which all
                                 of the statistical data will be written. */

    bool cuda_ = false;     /**< Flag used to indicate if cuda was used in the test */

    std::vector<visual_mtt::Tracks> tracks_data_; /**< Stores track data. This data constains the utilization data. */


  };

}