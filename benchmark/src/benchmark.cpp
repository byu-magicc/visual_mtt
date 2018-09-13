#include "benchmark/benchmark.h"

namespace visual_mtt_benchmark {

Benchmark::Benchmark() : nh_private_(ros::NodeHandle("~")) 
{

  sub_tracks_ = nh_private_.subscribe("/tracks",10,&Benchmark::CallbackTracks,this);

  nh_private_.param<std::string>("bench_file_path", file_path_, "example.csv");

#if OPENCV_CUDA
  cuda_ = true;
#endif


}

// ----------------------------------------------------------------------------------

Benchmark::~Benchmark()
{
  WriteCsvFile();
}

// ----------------------------------------------------------------------------------

void Benchmark::CallbackTracks(const visual_mtt::Tracks& msg) 
{
  tracks_data_.push_back(msg);
}

// ----------------------------------------------------------------------------------

void Benchmark::WriteCsvFile()
{
  // Used to get the average statistics. 
  float avg_num_tracks(0);
  float avg_feature_manager_util(0);
  float avg_homography_manager_util(0);
  float avg_measurement_generation_util(0);
  float avg_other_util(0);
  float avg_rransac_util(0);
  float avg_total_util(0);
  float avg_time_available(0);
  float avg_num_rransac_meas(0);

  std::ofstream bench_file;
  bench_file.open(file_path_);
  // bench_file << "Number of Tracks, Feature Manager Util, \
                 Homography Manager Util, Measurment Generation Util,\
                 RRANSAC Util, Total Util, Time Available, Number of RRANSAC Measurements\n";

  for (auto& track : tracks_data_) {

    // Write to file
    bench_file << track.tracks.size() << ","
               << track.util.feature_manager << ","
               << track.util.homography_manager << ","
               << track.util.measurement_generation << ","
               << track.util.other << ","
               << track.util.rransac << ","
               << track.util.total << ","
               << track.util.time_available << ","
               << track.util.number_of_rransac_measurements << "\n";

    // Sum statistics
    avg_num_tracks +=track.tracks.size();
    avg_feature_manager_util += track.util.feature_manager;
    avg_homography_manager_util += track.util.homography_manager;
    avg_measurement_generation_util += track.util.measurement_generation;
    avg_other_util += track.util.other;
    avg_rransac_util += track.util.rransac;
    avg_total_util += track.util.total;
    avg_time_available += track.util.time_available;
    avg_num_rransac_meas += track.util.number_of_rransac_measurements;

  }

  if (tracks_data_.size() > 0) {

    avg_num_tracks /= tracks_data_.size();
    avg_feature_manager_util /=tracks_data_.size();
    avg_homography_manager_util /=tracks_data_.size();
    avg_measurement_generation_util /=tracks_data_.size();
    avg_other_util /=tracks_data_.size();
    avg_rransac_util /=tracks_data_.size();
    avg_total_util /=tracks_data_.size();
    avg_time_available /=tracks_data_.size();
    avg_num_rransac_meas /=tracks_data_.size();

  }

  // Write to file
  // bench_file << " , , , , Averages, , , \n";
  bench_file << avg_num_tracks << ","
             << avg_feature_manager_util << ","
             << avg_homography_manager_util << ","
             << avg_measurement_generation_util << ","
             << avg_other_util << ","
             << avg_rransac_util << ","
             << avg_total_util << ","
             << avg_time_available << ","
             << avg_num_rransac_meas << "\n";

  // bench_file << "CUDA," << cuda_ << " , , , , , , \n";
  bench_file << cuda_ << " , , , , , , , \n";

  bench_file.close();


  // Print Data
  std::string title = "Visual MTT Benchmarking";
  int padding = 13;
  std::cout << std::endl;
  std::cout << std::string(2*padding+title.length(),'#') << std::endl;
  std::cout << '#' << std::string(padding-1,' ') << title << std::string(padding-1,' ') << "#\n";
  std::cout << std::string(2*padding+title.length(),'#') << std::endl;
  std::cout << "# CUDA: " << cuda_ << std::endl;
  std::cout << "# AVG NUM TRACKS: "                  << avg_num_tracks                         << std::endl;
  std::cout << "# AVG FEATURE MANAGER UTIL: "        << avg_feature_manager_util        << '%' << std::endl;
  std::cout << "# AVG HOMOGRAPHY MANAGER UTIL: "     << avg_homography_manager_util     << '%' << std::endl;
  std::cout << "# AVG MEASUREMENT GENERATION UTIL: " << avg_measurement_generation_util << '%' << std::endl;
  std::cout << "# AVG Other UTIL: "                  << avg_other_util                  << '%' << std::endl;
  std::cout << "# AVG RRANSAC UTIL: "                << avg_rransac_util                << '%' << std::endl;
  std::cout << "# AVG TOTAL UTIL: "                  << avg_total_util                  << '%' << std::endl;
  std::cout << "# AVG TIME AVAILABLE: "              << avg_time_available              << 's' <<std::endl;
  std::cout << "# AVG NUM RRANSAC MEAS: "            << avg_num_rransac_meas                   << std::endl;
  std::cout << std::string(2*padding+title.length(),'#') << std::endl;

}

}











