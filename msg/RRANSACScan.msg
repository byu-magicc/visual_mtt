# A R-RANSAC scan contains measurements from one or more sources as well as
# transform information that links the previous scan's coordinate frame
# to this one. For visual data, this transform is a perspective projectivity
# that is called a homography in the computer vision world. This message
# is intended to be sent from the visual frontend to the R-RANSAC node to
# feed into the R-RANSAC Tracker.

std_msgs/Header header_frame
std_msgs/Header header_scan

# Assumes Row-Major memory storage
float32[9] homography

visual_mtt2/Source[] sources