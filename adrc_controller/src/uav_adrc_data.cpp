//
// File: uav_adrc_data.cpp
//
// Code generated for Simulink model 'uav_adrc'.
//
// Model version                  : 1.4
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Jan 22 20:28:50 2026
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
  //    '<S25>/Gain2'
  //    '<S47>/Gain2'
  //    '<S69>/Gain2'

  { 1.0, 0.0, 0.0, 0.01, 1.0, 0.0, 5.0E-5, 0.01, 1.0 },

  // Pooled Parameter (Expression: bd)
  //  Referenced by:
  //    '<S25>/Gain3'
  //    '<S47>/Gain3'
  //    '<S69>/Gain3'

  { 5.0E-5, 0.01, 0.0 },

  // Pooled Parameter (Expression: ad*ld)
  //  Referenced by:
  //    '<S25>/Gain4'
  //    '<S47>/Gain4'
  //    '<S69>/Gain4'

  { 0.28548774589212145, 2.6736858796013676, 8.61784444348992 },

  // Pooled Parameter (Expression: ad)
  //  Referenced by:
  //    '<S14>/Gain2'
  //    '<S36>/Gain2'
  //    '<S58>/Gain2'

  { 1.0, 0.0, 0.0, 0.001, 1.0, 0.0, 5.0E-7, 0.001, 1.0 },

  // Pooled Parameter (Expression: bd)
  //  Referenced by:
  //    '<S14>/Gain3'
  //    '<S36>/Gain3'
  //    '<S58>/Gain3'

  { 5.0E-7, 0.001, 0.0 },

  // Pooled Parameter (Expression: ad*ld)
  //  Referenced by:
  //    '<S14>/Gain4'
  //    '<S36>/Gain4'
  //    '<S58>/Gain4'

  { 0.0594039800797342, 1.1723956402103148, 7.763952455012177 }
};

//
// File trailer for generated code.
//
// [EOF]
//
