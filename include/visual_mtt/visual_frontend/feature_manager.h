#include <iostream>
#include <opencv2/opencv.hpp>

class FeatureManager
{
public:
	FeatureManager();

	void find_correspondences(cv::Mat& prev_frame, cv::Mat& frame,
                            std::vector<cv::Point2f>& prev_features,
                            std::vector<cv::Point2f>& features,
                            std::vector<cv::Point2f>& diff_features,
                            cv::Mat& drawing, bool draw);

private:

	cv::Mat calibration_;

	// keep pyramids from the last iteration
	bool last_homograpy_good_;
	std::vector<cv::Mat> last_pyramids_;
	cv::Size pyr_size_;
	int pyr_size_param_;

};

