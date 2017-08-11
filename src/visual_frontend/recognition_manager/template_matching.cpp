#include "visual_frontend/recognition_manager/template_matching.h"

namespace visual_frontend {

TemplateMatching::TemplateMatching()
{

}

// ----------------------------------------------------------------------------

uint32_t TemplateMatching::identify_target(cv::Mat image)
{
  std::cout << "identifying requested track" << std::endl;
  return (uint32_t)0;
}

// ----------------------------------------------------------------------------

void TemplateMatching::update_descriptors(cv::Mat image, uint32_t idx)
{
  std::cout << "updating descriptors for " << idx << std::endl;
  return;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

}
