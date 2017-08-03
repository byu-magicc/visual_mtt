# The R-RANSAC Tracker supports measurement scans from multiple measurement
# sources. This message represents a single scan from a single source.
# It is included as part of the RRANSACScan message

# ID of the measurement source that found these measurements
uint8 id

# Measurement dimensionality. Use this to know how many
# elements are in the `position` and `velocity` arrays
uint8 dimensionality

# Whether or not this source produces velocity measurements
bool has_velocity

Measurement[] positions
Measurement[] velocities