# A single R-RANSAC Track (i.e., Good Model)

int32 id                        # Track ID (good model number)
float32 inlier_ratio            # Track inlier ratio (confidence)

geometry_msgs/Point position    # Position (x, y, z)
geometry_msgs/Point velocity    # Velocity (vx, vy, vz)
float32[] covariance            # Error covariance (row-major)