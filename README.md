Visual Multiple Target Tracker
==============================

This ROS package, known as `visual_mtt`, includes a visual measurement frontend that feeds the **R-RANSAC Tracker** (see [here](https://magiccvs.byu.edu/gitlab/robust_tracking/rransac)).

## Installation

1. Install the R-RANSAC Tracker (`librransac`) from [here](https://magiccvs.byu.edu/gitlab/robust_tracking/rransac).
2. Clone repo into `src` of a catkin workspace.
3. `catkin_make` in the root of your catkin workspace.

## Quick Start

To play a video file, use the following `roslaunch` command:

```bash
$ roslaunch visual_mtt2 play_from_recording.launch video_path:=/path/to/video.mp4 fps:=30
```

See the [Play From Recording](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/videos-and-rosbags) wiki page for a guide to using the `play_from_recording.launch` file for rosbags and videos.

## Tuning
When the `tuning` arg is set true in the main launch file, `visual_mtt.launch`, key parameters can be dynamically reconfigured using sliders. See the the detailed guide [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/tuning) to know what to look for. Once tuned, you can save a parameter .yaml for your specific application.

## Hardware Integration
Integrating `visual_mtt` with hardware is a matter of connecting the appropriate ROS topics, providing parameters, and disabling the display windows. See the detailed guide [here](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/hardware-integration)