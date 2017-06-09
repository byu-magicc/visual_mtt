#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_frontend/source.h"

class SourceBackground: public Source
{
public:
  SourceBackground();
  void generate_measurements(){std::cout << "specific source" << std::endl;}


private:


};