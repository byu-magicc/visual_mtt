# Utilization statistics from a single visual_mtt iteration.
# It is calculated as the ratio of the amount of time used
# in processing vs the amount of time available for processing.
#
#   utilization = t_computation / t_available *100

# utilization for different components of the visual measurement pipeline
float64 feature_manager
float64 homography_manager
float64 measurement_generation
float64 other

# utilization of the R-RANSAC Tracker
float64 rransac

# total utilization
float64 total

# time available: (1/fps)*stride
float64 time_available

# Number of measurements sent to R-RANSAC this scan
int32 number_of_rransac_measurements