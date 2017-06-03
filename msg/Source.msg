# The R-RANSAC Tracker supports measurement scans from multiple measurement
# sources. This message represents a single scan from a single source.
# It is included as part of the RRANSACScan message

# Do we need a name? Can't we just look it up using the source_id?
std_msgs/String name

# ID of the measurement source that found these measurements
uint8 source_id

# Does this measurement source produce velocity measurements?
bool has_velocity

# An array of measurement objects
visual_mtt2/Measurement[] measurements