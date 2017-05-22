#include "rransac/rransac.h"

namespace visual_mtt {

RRANSAC::RRANSAC()
{

	std::cout << "initializing rransac object inside node" << std::endl; // temporary
	tracker_ = rransac::Tracker(params_);
	std::cout << "initialized rransac object inside node" << std::endl; // temporary

	// ROS stuff
	// TODO discussion on the rostopic containing measurements/homographies (fix terrible topic name)
	sub = nh.subscribe("measurements_and_homographies", 1, &RRANSAC::callback, this); // noob question: is there a way to specify message type here? (in addition to specifying in the callback function)
	pub = nh.advertise<std_msgs::Float32>("rransac_tracks", 1); // temporary dummy std_msgs for compilation

}

void RRANSAC::callback(const std_msgs::Float32 data) // temporary dummy std_msgs for compilation
{
	// homographies and measurements arrive synchronized

	// separate homographies from measurements

	// apply measurements to tracker_

	// apply homographies to tracker_

	// retrieve good models and publish

}



}