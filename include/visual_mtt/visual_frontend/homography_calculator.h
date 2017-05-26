#include <iostream>
#include <opencv2/opencv.hpp>

class HomographyCalculator
{
public:
	HomographyCalculator();
	void calculate_homography(const std::vector<cv::Point2f>& prev_features,
                            const std::vector<cv::Point2f>& next_features);

  cv::Mat homography_;
  std::vector<uchar> inlier_mask_;

private:


};

