# Utilization statistics from a single visual_mtt iteration.
# It is calculated as the ratio of the amount of time used
# in processing vs the amount of time available for processing.
#
#   utilization = t_computation / t_available

# utilization for different components of the visual measurement pipeline
float64 feature_manager
float64 homography_manager
float64 measurement_generation

# utilization of the R-RANSAC Tracker
float64 rransac

# total utilization
float64 total

# Number of measurements sent to R-RANSAC this scan
int32 number_of_rransac_measurements