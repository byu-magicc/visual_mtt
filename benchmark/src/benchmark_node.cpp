#include <ros/ros.h>
#include "benchmark/benchmark.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "visual_frontend_benchmark_node");

  visual_mtt_benchmark::Benchmark benchmark;

  ros::spin();
  return 0;
}