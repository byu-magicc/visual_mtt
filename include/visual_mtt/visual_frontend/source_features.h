#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source_measurement.h"

class SourceFeatures: public SourceMeasurement
{
public:
	SourceFeatures();
	void generate_measurements(){std::cout << "specific source" << std::endl;}


private:


};

