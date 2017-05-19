#include "rransac/rransac.h"

namespace visual_mtt {

RRANSAC::RRANSAC()
{
	tracker_ = rransac::Tracker(params_);
}

}