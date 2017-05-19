#pragma once

#include <rransac/tracker.h>
#include <rransac/access_type.h>

namespace visual_mtt {

	class RRANSAC
	{
	public:
		RRANSAC();
		~RRANSAC();
		
	private:
		rransac::core::Parameters params_;
		rransac::Tracker tracker_;

	};

}