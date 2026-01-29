//
// File: uav_adrc_data.cpp
//
// Code generated for Simulink model 'uav_adrc'.
//
// Model version                  : 1.33
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Jan 29 11:39:39 2026
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
  //    '<S58>/Gain4'

  { 0.0089865134898809776, 0.026905702166335706, 0.026878803204032076 },

  // Expression: ad*ld
  //  Referenced by: '<S69>/Gain4'

  { 0.017946107838194156, 0.10724722933747736, 0.21406568510806817 },

  // Pooled Parameter (Expression: bd)
  //  Referenced by:
  //    '<S14>/Gain3'
  //    '<S36>/Gain3'

  { 2.1499999999999997E-5, 0.043000000000000003, 0.0 },

  // Pooled Parameter (Expression: ad*ld)
  //  Referenced by:
  //    '<S14>/Gain4'
  //    '<S36>/Gain4'

  { 0.074070263915002141, 1.8212757878504591, 15.051088591129274 },

  // Expression: bd
  //  Referenced by: '<S58>/Gain3'

  { 1.2499999999999999E-5, 0.025, 0.0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
