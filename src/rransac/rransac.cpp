#include "rransac/rransac.h"

namespace visual_mtt {

RRANSAC::RRANSAC()
{
  // instantiate the rransac::Tracker library class
  tracker_ = rransac::Tracker(params_);

  // ROS stuff
  sub = nh.subscribe("measurements", 1, &RRANSAC::callback, this);
  pub = nh.advertise<visual_mtt2::Tracks>("tracks", 1);
}

// ----------------------------------------------------------------------------

void RRANSAC::callback(const visual_mtt2::RRANSACScanPtr& rransac_scan)
{
  // Access the homography from the ROS message, convert to Projective2d, and give to R-RANSAC
  Eigen::Matrix3f H = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(rransac_scan->homography.data());
  Eigen::Projective2d T(H.cast<double>());
  tracker_.apply_transformation(T);


  // Add each source's measurements along with its ID to the R-RANSAC Tracker
  for (auto src = rransac_scan->sources.begin(); src != rransac_scan->sources.end(); src++)
    if (src->dimensionality == 2)
      tracker_.add_measurements<ROSVec2fAccess>(src->positions, src->velocities, src->id);
    else if (src->dimensionality == 3)
      tracker_.add_measurements<ROSVec3fAccess>(src->positions, src->velocities, src->id);


  // Run R-RANSAC and store any tracks (i.e., Good Models) to publish through ROS
  std::vector<rransac::core::ModelPtr> tracks = tracker_.run();


  // publish the tracks onto ROS network
  publish_tracks(tracks);

}

// ----------------------------------------------------------------------------

void RRANSAC::publish_tracks(const std::vector<rransac::core::ModelPtr>& tracks)
{

  // Create the ROS message we will send
  visual_mtt2::Tracks msg;
  
  for (int i=0; i<tracks.size(); i++)
  {
    visual_mtt2::Track track;

    // General track information
    track.id            = tracks[i]->GMN;
    track.inlier_ratio  = tracks[i]->rho;

    // Position measurements
    track.position.x    = tracks[i]->xhat(0);
    track.position.y    = tracks[i]->xhat(1);

    // Velocity measurements
    track.velocity.x    = tracks[i]->xhat(2);
    track.velocity.y    = tracks[i]->xhat(3);

    // Error covariance: Convert col-major double to row-major float  ---  #MakeDonaldDrumpfAgain
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> covfefe = tracks[i]->P.cast<float>();
    track.covariance.insert(track.covariance.end(), covfefe.data(), covfefe.data()+covfefe.size());

    // Add this track to the tracks msg
    msg.tracks.push_back(track);
  }

  // Add the current time to the tracks
  msg.timestamp = ros::Time::now();

  // ROS publish
  pub.publish(msg);
}

}