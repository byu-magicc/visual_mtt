Visual Multiple Target Tracker
==============================

This ROS package, known as `visual_mtt`, includes a visual measurement frontend that feeds the **R-RANSAC Tracker** (see [here](https://magiccvs.byu.edu/gitlab/robust_tracking/rransac)).

## Installation

1. Install the R-RANSAC Tracker (`librransac`) from [here](https://magiccvs.byu.edu/gitlab/robust_tracking/rransac).
1. Clone repo into `src` of a catkin workspace.
1. `catkin_make` in the root of your catkin workspace.

#### CUDA-Enabled Visual MTT

To speed up tracking using an NVIDIA CUDA GPGPU, you can build the `visual_mtt` ROS package with CUDA-enabled. In order to do this, you will need

1. An NVIDIA GPU (TX2, desktop machine, etc)
1. The CUDA toolkit must be installed.
1. OpenCV needs to be built from source with the `-DWITH_CUDA=ON` option (as well as others).
1. Make ROS play nicely with your custom install of OpenCV (managing catkin dependencies).

See the [MAGICC Wiki](https://magiccvs.byu.edu/wiki/#!sw_guides/opencv.md) for instructions and help with these steps.

Once you have your CUDA-enabled OpenCV environment setup, you can enable CUDA on `visual_mtt` with the following:

```bash
$ catkin_make -DOpenCV_DIR=/usr/local/share/OpenCV -DVISUAL_MTT_CUDA=ON
```

This tells `CMake` to look at *your* installation of OpenCV instead of the one at `/opt/ros/kinetic/share/OpenCV-3.2.0-dev`. Pay attention to the catkin_make process and you should see it tell you whether or not it found a CUDA-enabled OpenCV. Also, once you run the `visual_mtt` node, you should see it tell you how many CUDA devices it found.

## Quick Start

To play a video file, use the following `roslaunch` command:

```bash
$ roslaunch visual_mtt play_from_recording.launch video_path:=/path/to/video.mp4 fps:=30
```

See the [Play From Recording](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt/wikis/videos-and-rosbags) wiki page for a guide to using the `play_from_recording.launch` file for rosbags and videos.

## Tuning
When the `tuning` arg is set true in the main launch file, `visual_mtt.launch`, key parameters can be dynamically reconfigured using sliders. See the the detailed guide [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt/wikis/tuning) to know what to look for. Once tuned, you can save a parameter .yaml for your specific application.

## Hardware Integration
Integrating `visual_mtt` with hardware is a matter of connecting the appropriate ROS topics, providing parameters, and disabling the display windows. See the detailed guide [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt/wikis/hardware-integration)