#pragma once

#include <rransac/access_type.h>
#include <opencv2/opencv.hpp>

#include "visual_mtt2/Measurement.h"

namespace visual_mtt {

// For 2D measurements
class ROSVec2fAccess : public rransac::BaseAccessType<visual_mtt2::Measurement>
{
private:
    const double x(const visual_mtt2::Measurement& p) { return (double)p.data[0]; }
    const double y(const visual_mtt2::Measurement& p) { return (double)p.data[1]; }
};

// For 3D measurements
class ROSVec3fAccess : public rransac::BaseAccessType<visual_mtt2::Measurement>
{
private:
    const double x(const visual_mtt2::Measurement& p) { return (double)p.data[0]; }
    const double y(const visual_mtt2::Measurement& p) { return (double)p.data[1]; }
    const double z(const visual_mtt2::Measurement& p) { return (double)p.data[2]; }
};

}