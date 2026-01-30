//
// File: uav_adrc_data.cpp
//
// Code generated for Simulink model 'uav_adrc'.
//
// Model version                  : 1.35
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Fri Jan 30 19:44:29 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "uav_adrc.h"

// Constant parameters (default storage)
const uav_adrc::ConstP_uav_adrc_T uav_adrc_ConstP{
  // Pooled Parameter (Expression: ad)
  //  Referenced by:
  //    '<S14>/Gain2'
  //    '<S25>/Gain2'
  //    '<S36>/Gain2'
  //    '<S47>/Gain2'
  //    '<S58>/Gain2'
  //    '<S69>/Gain2'

  { 1.0, 0.0, 0.0, 0.001, 1.0, 0.0, 5.0E-7, 0.001, 1.0 },

  // Pooled Parameter (Expression: bd)
  //  Referenced by:
  //    '<S25>/Gain3'
  //    '<S47>/Gain3'
  //    '<S69>/Gain3'

  { 5.0E-7, 0.001, 0.0 },

  // Pooled Parameter (Expression: ad*ld)
  //  Referenced by:
  //    '<S25>/Gain4'
  //    '<S47>/Gain4'
  //    '<S69>/Gain4'

  { 0.014962562421953075, 0.074564058213000814, 0.12406639455918825 },

  // Pooled Parameter (Expression: bd)
  //  Referenced by:
  //    '<S14>/Gain3'
  //    '<S36>/Gain3'

  { 2.1499999999999997E-5, 0.043000000000000003, 0.0 },

  // Pooled Parameter (Expression: ad*ld)
  //  Referenced by:
  //    '<S14>/Gain4'
  //    '<S36>/Gain4'

  { 0.28548774589212145, 26.736858796013674, 861.784444348992 },

  // Expression: bd
  //  Referenced by: '<S58>/Gain3'

  { 1.2499999999999999E-5, 0.025, 0.0 },

  // Expression: ad*ld
  //  Referenced by: '<S58>/Gain4'

  { 0.020926671200294712, 0.14580547946467715, 0.33941942080752585 }
};

//
// File trailer for generated code.
//
// [EOF]
//
