#include <iostream>
#include <opencv2/opencv.hpp>

class FeatureManager
{
public:
	FeatureManager(
		bool compute_stats          = false,
		int max_points_tracked      = 2000,
		double corner_quality       = 0.03,
		double min_corner_quality   = 0.03,
		double max_corner_quality   = 0.5,
		double corner_quality_alpha = 0.999,
		int nominal_corner_count    = 1200,
		int pyr_size_param          = 21,
		int save_Nth_frame          = 0);

  void set_parameters();
	void find_correspondences(cv::Mat& frame);

private:

	bool first_image_;
	std::vector<cv::Point2f> last_features_;
	std::vector<cv::Point2f> next_matched_;
	cv::Mat last_image_;

	// Node parameters
	bool compute_stats_;
	int max_points_tracked_;
	double corner_quality_;
	double min_corner_quality_;
	double max_corner_quality_;
	double corner_quality_alpha_;
	int nominal_corner_count_;


	cv::Mat calibration_;
#if CV_MAJOR_VERSION == 2
	// we won't be supporting opencv2
	cv::Ptr<cv::GoodFeaturesToTrackDetector> gftt_detector;
#elif CV_MAJOR_VERSION == 3
	cv::Ptr<cv::GFTTDetector> gftt_detector_;
#endif

	// Optimization for optical flow: keep pyramids from the last iteration
	bool last_homograpy_good_;
	std::vector<cv::Mat> last_pyramids_;
	cv::Size pyr_size_;
	int pyr_size_param_;

	std::vector<cv::Point2i> track_count_;
	std::vector<cv::Point2i> track_count_nopredict_;

	unsigned int frame_count_;
	int save_Nth_frame_;

	// Settings
	cv::TermCriteria kltTerm_;

};

