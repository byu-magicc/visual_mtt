#pragma once

#include <opencv2/opencv.hpp>
#include <rransac/access_type.h>

namespace visual_frontend {

    /** \class CVPoint2fAccess
    * \brief Accessor to tell R-RANSAC how to interpret the data.
    */
class CVPoint2fAccess : public rransac::BaseAccessType<cv::Point2f>
{
private:
    const double x(const cv::Point2f& p) { return static_cast<double>(p.x); }
    const double y(const cv::Point2f& p) { return static_cast<double>(p.y); }
};

}