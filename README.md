Visual Multiple Target Tracker
==============================

This ROS package, known as `visual_mtt`, receives a sequence of images and camera parameters from which measurements are extracted and fed into the multiple target tracker Recursive-RANSAC (see [here](https://magiccvs.byu.edu/gitlab/recursive_ransac/rransac)). R-RANSAC uses the measurement to initialize and manage tracks in order to follow moving objects in the camera's field of view. 

## Installation

1. Install the R-RANSAC Tracker from [here](https://magiccvs.byu.edu/gitlab/recursive_ransac/rransac).
1. Install the Parallax Compensation library from [here](https://magiccvs.byu.edu/gitlab/robust_tracking/parallax_cpp).
1. Clone repo into `src` of a catkin workspace.
1. `catkin_make` in the root of your catkin workspace.

#### CUDA-Enabled Visual MTT

To speed up tracking using an NVIDIA CUDA GPGPU, you can build the `visual_mtt` ROS package with CUDA-enabled. In order to do this, you will need

1. An NVIDIA GPU (TX2, desktop machine, etc)
1. The CUDA toolkit must be installed.
1. OpenCV needs to be built from source with the `-DWITH_CUDA=ON` option (as well as others).
1. Make ROS play nicely with your custom install of OpenCV (managing catkin dependencies).
1. Clone [vision\_opencv](https://github.com/ros-perception/vision_opencv) into your workspace.

See the [MAGICC Wiki](https://magiccvs.byu.edu/wiki/#!sw_guides/opencv.md) for instructions and help with these steps.

Once you have your CUDA-enabled OpenCV environment setup, you can enable CUDA on `visual_mtt` with the following:

```bash
$ catkin_make -DOpenCV_DIR=/usr/local/opencv -DVISUAL_MTT_CUDA=ON
```

This tells `CMake` to look at *your* installation of OpenCV instead of the one at `/opt/ros/<release>/share/OpenCV-<version>-dev`. Pay attention to the catkin_make process and you should see it tell you whether or not it found a CUDA-enabled OpenCV. Also, once you run the `visual_mtt` node, you should see it tell you how many CUDA devices it found.

#### Tracking On Different Manifolds

The R-RANSAC library can perform multiple target tracking on different manifolds such as Euclidean space of 2-dimension, affine space of 2-dimension and others. Early versions of visual MTT only performed target tracking on Euclidean space of 2-dimension, but the current version allows tracking to be performed on the special Euclidean group of 2-dimension (SE(2)). 

By default, visual MTT tracks on the Euclidean space of 2-dimension. To build visuall MTT to track on SE(2), add the definition '-DTRACKING_SE2=ON'. In other words, when running catkin_make use the command. 

```bash
$ catkin_make -DTRACKING_SE2=ON
```
A word of caution. Tracking on SE(2) is designed to be done on the normalized virtual image plane due to the restrictions of the Homography when transforming measurements and tracks from the previous frame to the current frame. Thus, only build visual MTT to track on SE(2) if the camera is not moving or if the camera plane is parallel to the the plane the objects move on. 


## Quick Start

To play a video file, use the following `roslaunch` command:

```bash
$ roslaunch visual_mtt play_from_recording.launch video_path:=/path/to/video.mp4 fps:=30
```

To play a ROS bag, use the following `roslaunch` command:

```bash
$ roslaunch visual_mtt play_from_recording.launch bag_path:="/path/to/rosbag.bag" bag_topic:="<rostopic_name>"
```
or if your rosbag does not have a "camera_info" image transport topic:
```bash
$ roslaunch visual_mtt play_from_recording.launch bag_path:="/path/to/rosbag.bag" bag_topic:="<rostopic_name>" has_info:="false"
```
Notes: 
- If using image transport, the `bag_topic` argument `<rostopic_name>` should NOT include the suffix `/image_raw`
- The statement `small_object_tracking:="true"` can be appended to the above roslaunch commands to use a parameter set for small object detection.

See the [Play From Recording](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/videos-and-rosbags) wiki page for a guide to using the `play_from_recording.launch` file for rosbags and videos.

## Tuning
When the `tuning` arg is set true in the main launch file, `visual_mtt.launch`, key parameters can be dynamically reconfigured using sliders. See the the detailed guide [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt/wikis/tuning) to know what to look for. Once tuned, you can save a parameter .yaml for your specific application.

## Hardware Integration
Integrating `visual_mtt` with hardware is a matter of connecting the appropriate ROS topics, providing parameters, and disabling the display windows. See the detailed guide [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt/wikis/hardware-integration)

## Using Plugins
Visual MTT uses plugins to perform the core computation. See [Using Plugins](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/using-plugins) to learn how to specify which plugins you want
to use for your project.

## Saving Images
While in tuning mode, Visual MTT will display various pictures in windows generated by OpenCV. To save any of the pictures, double click on a window. These pictures will be saved in the folder 
specified by the launch file argument `picture_file_path`

## Benchmarking
The Visual MTT package includes a basic utilization benchmark. You can use this to check the installation or for other reasons. See [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/benchmarking) to learn how to benchmark with Visual MTT.

## Doxygen
Visual MTT uses Doxygen to generate html documentation. To generate it, navigate to the root directory of the project and run
``` bash
$ doxygen Doxyfile 
```
To view it, run `html/index.html` with your favorite web browser. For example
```bash
$ google-chrome html/index.html
```
