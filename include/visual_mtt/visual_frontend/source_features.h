#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source.h"

class SourceFeatures: public Source
{
public:
	SourceFeatures();
	void generate_measurements(){std::cout << "specific source" << std::endl;}


private:


};

