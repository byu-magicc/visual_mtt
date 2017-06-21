#pragma once

#include <rransac/access_type.h>

#include "visual_mtt/Measurement.h"

namespace rransac {

// For 2D measurements
class ROSVec2fAccess : public rransac::BaseAccessType<visual_mtt::Measurement>
{
private:
    const double x(const visual_mtt::Measurement& p) { return (double)p.data[0]; }
    const double y(const visual_mtt::Measurement& p) { return (double)p.data[1]; }
};

// For 3D measurements
class ROSVec3fAccess : public rransac::BaseAccessType<visual_mtt::Measurement>
{
private:
    const double x(const visual_mtt::Measurement& p) { return (double)p.data[0]; }
    const double y(const visual_mtt::Measurement& p) { return (double)p.data[1]; }
    const double z(const visual_mtt::Measurement& p) { return (double)p.data[2]; }
};

}