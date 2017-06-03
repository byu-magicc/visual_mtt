#pragma once

#include <rransac/access_type.h>
#include <opencv2/opencv.hpp>

namespace visual_mtt {

class Point2fAccess : public rransac::BaseAccessType<cv::Point2f>
{
private:
    const double x(const cv::Point2f& p) { return (double)p.x; }
    const double y(const cv::Point2f& p) { return (double)p.y; }
};

}