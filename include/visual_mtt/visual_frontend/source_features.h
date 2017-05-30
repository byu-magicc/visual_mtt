#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source.h"

// dynamic reconfig
#include "visual_mtt2/visual_frontendConfig.h"

class SourceFeatures: public Source
{
public:
	SourceFeatures();
	void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& features, std::vector<cv::Point2f>& features_vel);
  void set_parameters(visual_mtt2::visual_frontendConfig& config);

	bool first_image_ = true;

private:

	// needs cleanup! (some aren't being used)
	// also, any of these used by dynamic reconfigure will be overwritten
	// at launch - consider not assigning values for such.
	bool SHOW_OPENCV_VIEW_detectMovingPoints = 1;
	bool SHOW_OPENCV_VIEW_MOTION_MASK = 1;
	double MIN_FEATURE_DISTANCE  =1.;
	double MASK_THOLD_  =25;
	int MORPH_CLOSE_ITERATIONS_ = 1;
	int MORPH_KERNEL_SIZE_ = 2;
	double GAUSSIAN_BLUR_SIZE = 5;
	double GAUSSIAN_BLUR_SIGMA = 2;
	double MIN_PX_VEL = 1;
	double MAX_PX_VEL = 10.0;
	double homography_error_thold = 0.25;



};

