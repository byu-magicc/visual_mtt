#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

// this defines the base class for all measurement source classes
// a vector of Source objects can be used (polymorphism)

class Source
{
public:
  Source(){};
  ~Source(){};

  virtual void generate_measurements(){};
  virtual void set_parameters(){};

  // how to structure measurements? using an object might be overkill since
  // it will be immediately transformed to a ros message TODO: discussion.

private:
  std::string name;

};