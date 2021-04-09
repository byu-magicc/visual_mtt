#ifndef VISUAL_MTT_COMMON_OBJECT_TYPES_H_
#define VISUAL_MTT_COMMON_OBJECT_TYPES_H_
#pragma once

// This file declares the object types used in RRANSAC

#include <rransac/rransac.h>
#include <lie_groups/state.h>
#include <rransac/common/transformations/trans_homography.h>
// #include <rransac/common/transformations/transformation_null.h>
#include "rransac/common/data_association/validation_region_policies/validation_region_fixed_pos_policy.h"
#include "rransac/common/data_association/track_likelihood_info_policies/tli_ipdaf_policy.h"
#include "rransac/common/data_association/measurement_weight_policies/mw_ipdaf_policy.h"
#include <rransac/common/measurement/measurement_base.h>
#include <rransac/common/sources/source_base.h>
#include <rransac/visualization/draw_info.h>
#include <rransac/visualization/visualization_host.h>
#include <rransac/visualization/draw_meas_policies/draw_meas_R2_SE2_pos_policy.h>

#if TRACKING_SE2
  #include <rransac/common/sources/source_SEN_pos_vel.h>
  #include <rransac/common/models/model_SEN_pos_vel.h>
  #include <rransac/track_initialization/seed_policies/SE2_pos_seed_policy.h>
  #include <rransac/track_initialization/lmle_policies/nonlinear_lmle_policy.h> 
  #include <rransac/visualization/draw_track_policies/draw_track_policy_SE2.h>

#else // R2
  #include <rransac/common/sources/source_RN.h>
  #include <rransac/common/models/model_RN.h>
  #include <rransac/track_initialization/seed_policies/null_seed_policy.h>
  #include <rransac/track_initialization/lmle_policies/linear_lmle_policy.h>
  #include <rransac/visualization/draw_track_policies/draw_track_policy_R2.h>

#endif




#if TRACKING_SE2
  typedef rransac::RRANSACTemplateParameters<lie_groups::SE2_se2, rransac::SourceSENPosVel,rransac::TransformHomography, rransac::ModelSENPosVel,rransac::SE2PosSeedPolicy,rransac::NonLinearLMLEPolicy,rransac::ValidationRegionFixedPosPolicy, rransac::TLI_IPDAFPolicy,rransac::MW_IPDAFPolicy> RRTemplateParams;

#else
  typedef rransac::RRANSACTemplateParameters<lie_groups::R2_r2, rransac::SourceRN,rransac::TransformHomography, rransac::ModelRN,rransac::NULLSeedPolicy,rransac::LinearLMLEPolicy,rransac::ValidationRegionFixedPosPolicy, rransac::TLI_IPDAFPolicy,rransac::MW_IPDAFPolicy> RRTemplateParams;
#endif

typedef typename RRTemplateParams::Model RR_Model;
typedef typename RRTemplateParams::State RR_State;
typedef typename rransac::System<RR_Model> RR_System;
typedef typename rransac::RRANSAC<RRTemplateParams> RR_RRANSAC;




#endif // VISUAL_MTT_COMMON_OBJECT_TYPES_H_
