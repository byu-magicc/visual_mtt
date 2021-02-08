cmake_minimum_required(VERSION 3.1)
project(visual_mtt)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  image_transport
  pluginlib
  camera_info_manager
)

# yaml-cpp is used to read in a yaml file and retrieve its values 
find_package(yaml-cpp REQUIRED)
# gnsac is used to optimize the essential matrix using RANSAC or LMEDS.
find_package(gnsac REQUIRED)

find_package(Eigen3)
find_package(rransac 1.0.4 EXACT REQUIRED)
message(STATUS "R-RANSAC Version: " ${RRANSAC_VERSION})
find_package(OpenCV 4)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version: ${OpenCV_VERSION} (${OpenCV_DIR})")
if(OpenCV_VERSION VERSION_LESS 3.0.0)
  message(FATAL_ERROR "OpenCV version must be at least 3")
endif()

option(VISUAL_MTT_CUDA "Enable CUDA support for the R-RANSAC visual measurement frontend" OFF)
if(VISUAL_MTT_CUDA)
  message(STATUS "==> Visual MTT with CUDA-enabled OpenCV requested")
  if(OpenCV_CUDA_VERSION)
    message(STATUS "+++ CUDA-enabled OpenCV found, CUDA Version: " ${OpenCV_CUDA_VERSION})
    add_definitions(-DOPENCV_CUDA)
  else()
    string(ASCII 27 ESC)
    message(STATUS "${ESC}[1;31m+++${ESC}[0m" " CUDA-enabled OpenCV not found")
  endif()

  # If you are using visual_MTT_CUDA, you cannot be using the opencv libraries
  # installed with ROS since they are not build with CUDA support.
  # These libraries can be added to the list ${catkin_LIBRARIES}
  # if you are using the cv_bridge package installed with ROS since
  # this package is linked to the opencv libraries installed by ROS.
  # To avoid this problem, you need to build your own cv_package
  # with the proper opencv libraries in the same workspace as visual_mtt.
  #
  # This loop makes sure you are not using the opencv libraries installed
  # with ROS if you are building with CUDA support. 
  foreach(loop_var ${catkin_LIBRARIES})
    string(REGEX MATCHALL "libopencv" OPENCV_LIB ${loop_var})
    if( ${OPENCV_LIB} STREQUAL ${OPENCV_LIB} )
      string(REGEX MATCHALL "ros" ROS_CHECK ${loop_var})
        if(${ROS_CHECK} STREQUAL ${ROS_CHECK})
          message(SEND_ERROR "Using ROS installed OpenCV libraries which do not have CUDA support. Ensure that you are not using the ROS installed cv_bridge package. See README.md for more information.")
        endif()
      # set(cv_libs ${cv_libs} ${loop_var})
      LIST(REMOVE_ITEM catkin_LIBRARIES ${loop_var} )
    endif()
  endforeach()


endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Tracks.msg
  Track.msg
  Utilization.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/visual_frontend.cfg
  cfg/rransac.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS cv_bridge message_runtime pluginlib
#  INCLUDE_DIRS include
#  LIBRARIES visual_mtt
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

message( STATUS "${RRANSAC_INCLUDE_DIRS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/visual_mtt 
  ${catkin_INCLUDE_DIRS}
  ${RRANSAC_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${GNSAC_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

## Plugins
add_library(measurement_source_plugin 
  src/visual_frontend/measurement_manager/feature_motion.cpp
  src/visual_frontend/measurement_manager/difference_image.cpp
  src/visual_frontend/measurement_manager/color_detector.cpp)

add_library(transform_method_plugin 
  src/visual_frontend/transform_manager/simple_homography.cpp)

add_library(feature_tracker_plugin 
  src/visual_frontend/feature_manager/lkt_tracker.cpp)

add_dependencies(measurement_source_plugin ${PROJECT_NAME}_gencfg)
add_dependencies(transform_method_plugin ${PROJECT_NAME}_gencfg)
add_dependencies(feature_tracker_plugin ${PROJECT_NAME}_gencfg)



target_link_libraries(measurement_source_plugin
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS})

target_link_libraries(transform_method_plugin
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS})

target_link_libraries(feature_tracker_plugin
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_visual_frontend
  src/visual_frontend/node.cpp
  src/visual_frontend/visual_frontend.cpp

  # Managers
  src/visual_frontend/feature_manager/feature_manager.cpp
  src/visual_frontend/transform_manager/transform_manager.cpp
  src/visual_frontend/measurement_manager/measurement_manager.cpp

  # Target recognition
  src/visual_frontend/recognition_manager/recognition_manager.cpp
  src/visual_frontend/recognition_manager/template_matching.cpp

  # Common
  src/common/gpu.cpp
  src/common/system.cpp)


add_executable(${PROJECT_NAME}_camera_sim
  src/camera_sim/node.cpp
  src/camera_sim/camera_sim.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_visual_frontend PROPERTIES OUTPUT_NAME visual_frontend PREFIX "")
set_target_properties(${PROJECT_NAME}_camera_sim PROPERTIES OUTPUT_NAME camera_sim PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_visual_frontend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_camera_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_visual_frontend 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${RRANSAC_LIBS}
  ${GNSAC_LIBS}
  ${YAML_CPP_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_camera_sim ${catkin_LIBRARIES} ${OpenCV_LIBS})


############
# Darknet  #
############
option(BUILD_DARKNET "Build the darknet plugin" OFF)
if(BUILD_DARKNET)
  
  add_subdirectory(darknet_wrapper)

  add_definitions( -DVMTT_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}")
  include_directories(  
    darknet_wrapper/include
    darknet_wrapper/darknet/include
    darknet_wrapper/darknet/src)
  target_sources(measurement_source_plugin PUBLIC
    src/visual_frontend/measurement_manager/darknet_plugin.cpp)
  add_dependencies(measurement_source_plugin darknet_wrapper)

  target_link_libraries(measurement_source_plugin
  darknet_wrapper)
endif()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

##################
## Benchmarking ##
##################
option(BUILD_BENCHMARK "Benchmarking for Visual_MTT" OFF)

if(BUILD_BENCHMARK)

add_subdirectory(benchmark)

endif()

#############
## Testing ##
#############


option(BUILD_TESTS "Build the Params GTests" OFF)
if(BUILD_TESTS)
  add_subdirectory(test)
endif()


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_visual_mtt.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
