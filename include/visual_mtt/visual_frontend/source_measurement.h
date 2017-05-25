#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// this defines the base class for all measurement source classes
// a vector of SourceMeasurement objects can be used

// forward declare VisualFrontend    // rh
namespace visual_mtt {               // rh
class VisualFrontend;                // rh
}                                    // rh

typedef void (visual_mtt::VisualFrontend::*methodPtr)(int);

class SourceMeasurement
{
public:
  SourceMeasurement(){};
  ~SourceMeasurement(){};

  virtual void generate_measurements(){};
  virtual void set_parameters(){};

  visual_mtt::VisualFrontend* saved_owner_;                                     // rh
  void add_handle(visual_mtt::VisualFrontend* owner){saved_owner_ = owner;};    // rh

  // how to structure measurements? using an object might be overkill since
  // it will be immediately transformed to a ros message

private:

};