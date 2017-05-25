#include <iostream>
#include <opencv2/opencv.hpp>

class HomographyCalculator
{
public:
	HomographyCalculator();
	void calculate_homography(std::vector<cv::Point2f>& prev_features,
                            std::vector<cv::Point2f>& features);

private:


};

