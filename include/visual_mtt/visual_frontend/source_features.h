#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source.h"

class SourceFeatures: public Source
{
public:
	SourceFeatures();
	void generate_measurements(cv::Mat& homography, std::vector<cv::Point2f>& features, std::vector<cv::Point2f>& features_vel);

	bool first_image_ = true;

private:


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

