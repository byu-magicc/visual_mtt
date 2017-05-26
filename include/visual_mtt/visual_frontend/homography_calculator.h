#include <iostream>
#include <opencv2/opencv.hpp>

class HomographyCalculator
{
public:
	HomographyCalculator();
	void calculate_homography(std::vector<cv::Point2f>& prev_features,
                            std::vector<cv::Point2f>& next_features);

  cv::Mat homography_;
  std::vector<uchar> inlier_mask_;

private:


};

