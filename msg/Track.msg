# A single R-RANSAC Track (i.e., Good Model)

int32 id                        # Track ID (good model number)
float32 model_likelihood        # The likelihood that a model represents a target (confidence)

geometry_msgs/Point position    # Position (x, y, z)
float32[] R                       # If tracking is done on SE2, then this is the passive rotation from the body to the inertial frame.
                                # The values will be loaded into the vector as {R11, R12, R21, R22}. If tracking is done
                                # on R2, then this value will never be set. 
geometry_msgs/Point velocity    # Velocity (vx, vy, vz). If tracking is done on SE2, then velocity is in the body frame
float32 omega                   # If tracking is done on SE2, this will be set to the angular velocity



float32[] covariance            # Error covariance (row-major)