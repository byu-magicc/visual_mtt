Visual Multiple Target Tracker
==============================

This ROS package includes a visual measurement frontend that feeds the **R-RANSAC Tracker** (see [here](https://magiccvs.byu.edu/gitlab/robust_tracking/rransac)).

See the [Pre-Recorded Video](https://magiccvs.byu.edu/gitlab/robust_tracking/visual_mtt2/wikis/videos-and-rosbags) wiki page for a guide to using the `play_from_recording.launch` file.

## Installation

1. Install `rransaclib`
2. Clone repo into `src` of a catkin workspace.
3. 


## Philosophies
(for early development)

* Automatic CUDA: run the exact same codebase for CUDA-enabled and CUDA-denied applications. [Simple Exapmle](https://github.com/jdmillard/opencv-cuda).
* Benchmarking (ongoing discussion). Idea: live computation profiling under the hood at all times. This would be different than benchmarking because it would provide specific information about key segments of the algorithm.
* Create the visual_frontend and rransac nodes to be generic and modular. Then have launchfiles remap topics and set appropriate flags. This way, the order of nodes can be rearranged for specific applications.
* Clean timestamps. Final tracks that are published will have two(?) timestamps. The first timestamp will be that of the frame from which the updates were processed, the second will be the time at which the tracks were published. This ought to be discussed.
* Provide options for features (GFTT, ORB)
* Expand the `Nth frame` functionality to support decimation and "sliding". This would provide higher update rates while still separating frames enough to detect movement. Decimation and sliding params would be chosen based on computational limitations. Homography calculation would need to take this into account.
* Clean IMU implementation (needs details)
* Ability to see visualization of measurements generated from each source as well as dynamic reconfiguration of thresholds.