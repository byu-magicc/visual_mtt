#include "visual_frontend/recognition_manager/template_matching.h"

namespace visual_frontend {

TemplateMatching::TemplateMatching()
{
  // TODO
}

// ----------------------------------------------------------------------------

uint32_t TemplateMatching::identify_target(cv::Mat image)
{
  // use history and current image to identify target
  // TODO
  return (uint32_t)0;
}

// ----------------------------------------------------------------------------

void TemplateMatching::update_descriptors(cv::Mat image, uint32_t idx)
{
  // use id and current image to update the descriptors
  // TODO
  return;
}

// ----------------------------------------------------------------------------

void TemplateMatching::set_parameters(visual_mtt::visual_frontendConfig& config)
{
  // update parameters
  // TODO
  return;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

}
