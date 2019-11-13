# See if video_stream_opencv is installed on the system. If not,
# then install the package. 
find_package(video_stream_opencv QUIET)

message(STATUS "Building Benchmarks")


if(NOT ${video_stream_opencv_FOUND})
  message(WARNING "You will need to install ros-$ENV{ROS_DISTRO}-video-stream-opencv")
endif()

# Download darknet test video file from MOT website. 
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/video")
set(FILE "${PATH}/MOT17-11-SDP.mp4")
message(STATUS "Checking and downloading MOT17-11-SDP.mp4 if needed ...")
if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget https://motchallenge.net/movies/MOT17-11-SDP.mp4 -P ${PATH})
endif()

include_directories(include)

add_executable(${PROJECT_NAME}_benchmark 
  src/benchmark.cpp 
  src/benchmark_node.cpp)

# set_target_properties(${PROJECT_NAME}_benchmark PROPERTIES OUTPUT_NAME benchmark PREFIX "")
add_dependencies(${PROJECT_NAME}_benchmark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_benchmark ${catkin_LIBRARIES})



