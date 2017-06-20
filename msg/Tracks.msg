# At each iteration, the R-RANSAC Tracker outputs a
# set of tracks using this message

# provide the time of the original frame and after Kalman update
# the elapsed time is total cost of the visual frontend and R-RANSAC
std_msgs/Header header_frame
std_msgs/Header header_update

Track[] tracks