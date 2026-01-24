//
// File: uav_adrc.cpp
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
#include <emmintrin.h>
#include "rtwtypes.h"

static void rate_scheduler(uav_adrc::RT_MODEL_uav_adrc_T *const uav_adrc_M);

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(uav_adrc::RT_MODEL_uav_adrc_T *const uav_adrc_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (uav_adrc_M->Timing.TaskCounters.TID[1])++;
  if ((uav_adrc_M->Timing.TaskCounters.TID[1]) > 9) {// Sample time: [0.01s, 0.0s] 
    uav_adrc_M->Timing.TaskCounters.TID[1] = 0;
  }
}

// Model step function
void uav_adrc::step()
{
  real_T rtb_Gain4[3];
  real_T tmp_0[3];
  real_T UnitDelay_DSTATE;
  real_T UnitDelay_DSTATE_0;
  real_T UnitDelay_DSTATE_1;
  real_T rtb_Gain2_h;
  real_T rtb_Sum1;
  int32_T i;
  boolean_T tmp;
  tmp = ((&uav_adrc_M)->Timing.TaskCounters.TID[1] == 0);
  if (tmp) {
    // Sum: '<S25>/Subtract' incorporates:
    //   Gain: '<S25>/Gain'
    //   Inport: '<Root>/X Position State'
    //   UnitDelay: '<S25>/Unit Delay'

    rtb_Gain2_h = uav_adrc_U.XPositionState - ((0.0 *
      uav_adrc_DW.UnitDelay_DSTATE[1] + uav_adrc_DW.UnitDelay_DSTATE[0]) + 0.0 *
      uav_adrc_DW.UnitDelay_DSTATE[2]);

    // Sum: '<S24>/Sum1' incorporates:
    //   Gain: '<S24>/Gain2'
    //   Gain: '<S25>/Gain1'
    //   Inport: '<Root>/X Postion Ref '
    //   Sum: '<S24>/Sum'
    //   Sum: '<S24>/Sum2'
    //   Sum: '<S25>/Add'
    //   UnitDelay: '<S25>/Unit Delay'

    rtb_Sum1 = ((uav_adrc_U.XPostionRef - (0.25918177931828223 * rtb_Gain2_h +
      uav_adrc_DW.UnitDelay_DSTATE[0])) - (2.5875074351664682 * rtb_Gain2_h +
      uav_adrc_DW.UnitDelay_DSTATE[1]) * 2.0) - (8.61784444348992 * rtb_Gain2_h
      + uav_adrc_DW.UnitDelay_DSTATE[2]);

    // UnitDelay: '<S25>/Unit Delay' incorporates:
    //   Gain: '<S25>/Gain2'

    UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE[1];
    UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE[0];
    UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE[2];

    // Sum: '<S25>/Add1' incorporates:
    //   Gain: '<S25>/Gain2'
    //   Gain: '<S25>/Gain3'
    //   Gain: '<S25>/Gain4'
    //   UnitDelay: '<S25>/Unit Delay'

    for (i = 0; i <= 0; i += 2) {
      _mm_storeu_pd(&tmp_0[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd(&uav_adrc_ConstP.pooled2[i + 3]), _mm_set1_pd
         (UnitDelay_DSTATE)), _mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled2[i]),
        _mm_set1_pd(UnitDelay_DSTATE_0))), _mm_mul_pd(_mm_loadu_pd
        (&uav_adrc_ConstP.pooled2[i + 6]), _mm_set1_pd(UnitDelay_DSTATE_1))),
        _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled6[i]),
        _mm_set1_pd(rtb_Sum1)), _mm_mul_pd(_mm_loadu_pd
        (&uav_adrc_ConstP.pooled7[i]), _mm_set1_pd(rtb_Gain2_h)))));
    }

    for (i = 2; i < 3; i++) {
      tmp_0[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
                   uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
                  uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
        (uav_adrc_ConstP.pooled6[i] * rtb_Sum1 + uav_adrc_ConstP.pooled7[i] *
         rtb_Gain2_h);
    }

    uav_adrc_DW.UnitDelay_DSTATE[0] = tmp_0[0];
    uav_adrc_DW.UnitDelay_DSTATE[1] = tmp_0[1];
    uav_adrc_DW.UnitDelay_DSTATE[2] = tmp_0[2];

    // End of Sum: '<S25>/Add1'

    // Outport: '<Root>/X Acceleration'
    uav_adrc_Y.XAcceleration = rtb_Sum1;
  }

  // Sum: '<S57>/Sum1' incorporates:
  //   Gain: '<S14>/Gain'
  //   Inport: '<Root>/X Attitude State'
  //   Sum: '<S14>/Subtract'
  //   UnitDelay: '<S14>/Unit Delay'

  uav_adrc_Y.ZTorque = uav_adrc_U.XAttitudeState - ((0.0 *
    uav_adrc_DW.UnitDelay_DSTATE_a[1] + uav_adrc_DW.UnitDelay_DSTATE_a[0]) + 0.0
    * uav_adrc_DW.UnitDelay_DSTATE_a[2]);

  // Sum: '<S13>/Sum1' incorporates:
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/Gain2'
  //   Gain: '<S14>/Gain1'
  //   Inport: '<Root>/X Attitude Ref'
  //   Sum: '<S13>/Sum'
  //   Sum: '<S13>/Sum2'
  //   Sum: '<S14>/Add'
  //   UnitDelay: '<S14>/Unit Delay'

  rtb_Sum1 = ((uav_adrc_U.XAttitudeRef - (0.058235466415751391 *
    uav_adrc_Y.ZTorque + uav_adrc_DW.UnitDelay_DSTATE_a[0])) * 4.0 -
              (1.1646316877553027 * uav_adrc_Y.ZTorque +
               uav_adrc_DW.UnitDelay_DSTATE_a[1]) * 4.0) - (7.763952455012177 *
    uav_adrc_Y.ZTorque + uav_adrc_DW.UnitDelay_DSTATE_a[2]);

  // UnitDelay: '<S14>/Unit Delay' incorporates:
  //   Gain: '<S14>/Gain2'

  rtb_Gain2_h = uav_adrc_DW.UnitDelay_DSTATE_a[1];
  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_a[0];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_a[2];

  // Sum: '<S14>/Add1' incorporates:
  //   Gain: '<S14>/Gain2'
  //   Gain: '<S14>/Gain3'
  //   Gain: '<S14>/Gain4'
  //   UnitDelay: '<S14>/Unit Delay'

  for (i = 0; i <= 0; i += 2) {
    _mm_storeu_pd(&tmp_0[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&uav_adrc_ConstP.pooled8[i + 3]), _mm_set1_pd(rtb_Gain2_h)),
      _mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled8[i]), _mm_set1_pd
                 (UnitDelay_DSTATE))), _mm_mul_pd(_mm_loadu_pd
      (&uav_adrc_ConstP.pooled8[i + 6]), _mm_set1_pd(UnitDelay_DSTATE_0))),
      _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled11[i]),
      _mm_set1_pd(rtb_Sum1)), _mm_mul_pd(_mm_loadu_pd
      (&uav_adrc_ConstP.pooled12[i]), _mm_set1_pd(uav_adrc_Y.ZTorque)))));
  }

  for (i = 2; i < 3; i++) {
    tmp_0[i] = ((uav_adrc_ConstP.pooled8[i + 3] * rtb_Gain2_h +
                 uav_adrc_ConstP.pooled8[i] * UnitDelay_DSTATE) +
                uav_adrc_ConstP.pooled8[i + 6] * UnitDelay_DSTATE_0) +
      (uav_adrc_ConstP.pooled11[i] * rtb_Sum1 + uav_adrc_ConstP.pooled12[i] *
       uav_adrc_Y.ZTorque);
  }

  uav_adrc_DW.UnitDelay_DSTATE_a[0] = tmp_0[0];
  uav_adrc_DW.UnitDelay_DSTATE_a[1] = tmp_0[1];
  uav_adrc_DW.UnitDelay_DSTATE_a[2] = tmp_0[2];

  // End of Sum: '<S14>/Add1'

  // Outport: '<Root>/X Torque'
  uav_adrc_Y.XTorque = rtb_Sum1;
  if (tmp) {
    // Sum: '<S47>/Subtract' incorporates:
    //   Gain: '<S47>/Gain'
    //   Inport: '<Root>/Y Position State'
    //   UnitDelay: '<S47>/Unit Delay'

    rtb_Gain2_h = uav_adrc_U.YPositionState - ((0.0 *
      uav_adrc_DW.UnitDelay_DSTATE_b[1] + uav_adrc_DW.UnitDelay_DSTATE_b[0]) +
      0.0 * uav_adrc_DW.UnitDelay_DSTATE_b[2]);

    // Sum: '<S46>/Sum1' incorporates:
    //   Gain: '<S46>/Gain2'
    //   Gain: '<S47>/Gain1'
    //   Inport: '<Root>/Y Postion Ref'
    //   Sum: '<S46>/Sum'
    //   Sum: '<S46>/Sum2'
    //   Sum: '<S47>/Add'
    //   UnitDelay: '<S47>/Unit Delay'

    rtb_Sum1 = ((uav_adrc_U.YPostionRef - (0.25918177931828223 * rtb_Gain2_h +
      uav_adrc_DW.UnitDelay_DSTATE_b[0])) - (2.5875074351664682 * rtb_Gain2_h +
      uav_adrc_DW.UnitDelay_DSTATE_b[1]) * 2.0) - (8.61784444348992 *
      rtb_Gain2_h + uav_adrc_DW.UnitDelay_DSTATE_b[2]);

    // UnitDelay: '<S47>/Unit Delay' incorporates:
    //   Gain: '<S47>/Gain2'

    UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_b[1];
    UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_b[0];
    UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_b[2];

    // Sum: '<S47>/Add1' incorporates:
    //   Gain: '<S47>/Gain2'
    //   Gain: '<S47>/Gain3'
    //   Gain: '<S47>/Gain4'
    //   UnitDelay: '<S47>/Unit Delay'

    for (i = 0; i <= 0; i += 2) {
      _mm_storeu_pd(&tmp_0[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd(&uav_adrc_ConstP.pooled2[i + 3]), _mm_set1_pd
         (UnitDelay_DSTATE)), _mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled2[i]),
        _mm_set1_pd(UnitDelay_DSTATE_0))), _mm_mul_pd(_mm_loadu_pd
        (&uav_adrc_ConstP.pooled2[i + 6]), _mm_set1_pd(UnitDelay_DSTATE_1))),
        _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled6[i]),
        _mm_set1_pd(rtb_Sum1)), _mm_mul_pd(_mm_loadu_pd
        (&uav_adrc_ConstP.pooled7[i]), _mm_set1_pd(rtb_Gain2_h)))));
    }

    for (i = 2; i < 3; i++) {
      tmp_0[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
                   uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
                  uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
        (uav_adrc_ConstP.pooled6[i] * rtb_Sum1 + uav_adrc_ConstP.pooled7[i] *
         rtb_Gain2_h);
    }

    // Outport: '<Root>/Y Acceleration'
    uav_adrc_Y.YAcceleration = rtb_Sum1;

    // Sum: '<S47>/Add1' incorporates:
    //   UnitDelay: '<S47>/Unit Delay'

    uav_adrc_DW.UnitDelay_DSTATE_b[0] = tmp_0[0];
    uav_adrc_DW.UnitDelay_DSTATE_b[1] = tmp_0[1];
    uav_adrc_DW.UnitDelay_DSTATE_b[2] = tmp_0[2];

    // Sum: '<S69>/Subtract' incorporates:
    //   Gain: '<S69>/Gain'
    //   Inport: '<Root>/Z Position State'
    //   UnitDelay: '<S69>/Unit Delay'

    rtb_Gain2_h = uav_adrc_U.ZPositionState - ((0.0 *
      uav_adrc_DW.UnitDelay_DSTATE_am[1] + uav_adrc_DW.UnitDelay_DSTATE_am[0]) +
      0.0 * uav_adrc_DW.UnitDelay_DSTATE_am[2]);

    // Sum: '<S68>/Sum1' incorporates:
    //   Gain: '<S68>/Gain2'
    //   Gain: '<S69>/Gain1'
    //   Inport: '<Root>/Z Postion Ref '
    //   Sum: '<S68>/Sum'
    //   Sum: '<S68>/Sum2'
    //   Sum: '<S69>/Add'
    //   UnitDelay: '<S69>/Unit Delay'

    rtb_Sum1 = ((uav_adrc_U.ZPostionRef - (0.25918177931828223 * rtb_Gain2_h +
      uav_adrc_DW.UnitDelay_DSTATE_am[0])) - (2.5875074351664682 * rtb_Gain2_h +
      uav_adrc_DW.UnitDelay_DSTATE_am[1]) * 2.0) - (8.61784444348992 *
      rtb_Gain2_h + uav_adrc_DW.UnitDelay_DSTATE_am[2]);

    // UnitDelay: '<S69>/Unit Delay' incorporates:
    //   Gain: '<S69>/Gain2'

    UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_am[1];
    UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_am[0];
    UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_am[2];

    // Sum: '<S69>/Add1' incorporates:
    //   Gain: '<S69>/Gain2'
    //   Gain: '<S69>/Gain3'
    //   Gain: '<S69>/Gain4'
    //   UnitDelay: '<S69>/Unit Delay'

    for (i = 0; i <= 0; i += 2) {
      _mm_storeu_pd(&rtb_Gain4[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd(&uav_adrc_ConstP.pooled2[i + 3]), _mm_set1_pd
         (UnitDelay_DSTATE)), _mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled2[i]),
        _mm_set1_pd(UnitDelay_DSTATE_0))), _mm_mul_pd(_mm_loadu_pd
        (&uav_adrc_ConstP.pooled2[i + 6]), _mm_set1_pd(UnitDelay_DSTATE_1))),
        _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled6[i]),
        _mm_set1_pd(rtb_Sum1)), _mm_mul_pd(_mm_loadu_pd
        (&uav_adrc_ConstP.pooled7[i]), _mm_set1_pd(rtb_Gain2_h)))));
    }

    for (i = 2; i < 3; i++) {
      rtb_Gain4[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
                       uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
                      uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
        (uav_adrc_ConstP.pooled6[i] * rtb_Sum1 + uav_adrc_ConstP.pooled7[i] *
         rtb_Gain2_h);
    }

    // End of Sum: '<S69>/Add1'

    // Outport: '<Root>/Z Acceleration'
    uav_adrc_Y.ZAcceleration = rtb_Sum1;
  }

  // Sum: '<S36>/Subtract' incorporates:
  //   Gain: '<S36>/Gain'
  //   Inport: '<Root>/Y Attitude State'
  //   UnitDelay: '<S36>/Unit Delay'

  rtb_Sum1 = uav_adrc_U.YAttitudeState - ((0.0 * uav_adrc_DW.UnitDelay_DSTATE_f
    [1] + uav_adrc_DW.UnitDelay_DSTATE_f[0]) + 0.0 *
    uav_adrc_DW.UnitDelay_DSTATE_f[2]);

  // Sum: '<S57>/Sum1' incorporates:
  //   Gain: '<S35>/Gain'
  //   Gain: '<S35>/Gain2'
  //   Gain: '<S36>/Gain1'
  //   Inport: '<Root>/Y Attitude Ref'
  //   Sum: '<S35>/Sum'
  //   Sum: '<S35>/Sum1'
  //   Sum: '<S35>/Sum2'
  //   Sum: '<S36>/Add'
  //   UnitDelay: '<S36>/Unit Delay'

  uav_adrc_Y.ZTorque = ((uav_adrc_U.YAttitudeRef - (0.058235466415751391 *
    rtb_Sum1 + uav_adrc_DW.UnitDelay_DSTATE_f[0])) * 4.0 - (1.1646316877553027 *
    rtb_Sum1 + uav_adrc_DW.UnitDelay_DSTATE_f[1]) * 4.0) - (7.763952455012177 *
    rtb_Sum1 + uav_adrc_DW.UnitDelay_DSTATE_f[2]);

  // UnitDelay: '<S36>/Unit Delay' incorporates:
  //   Gain: '<S36>/Gain2'

  rtb_Gain2_h = uav_adrc_DW.UnitDelay_DSTATE_f[1];
  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_f[0];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_f[2];

  // Sum: '<S36>/Add1' incorporates:
  //   Gain: '<S36>/Gain2'
  //   Gain: '<S36>/Gain3'
  //   Gain: '<S36>/Gain4'
  //   UnitDelay: '<S36>/Unit Delay'

  for (i = 0; i <= 0; i += 2) {
    _mm_storeu_pd(&tmp_0[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&uav_adrc_ConstP.pooled8[i + 3]), _mm_set1_pd(rtb_Gain2_h)),
      _mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled8[i]), _mm_set1_pd
                 (UnitDelay_DSTATE))), _mm_mul_pd(_mm_loadu_pd
      (&uav_adrc_ConstP.pooled8[i + 6]), _mm_set1_pd(UnitDelay_DSTATE_0))),
      _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled11[i]),
      _mm_set1_pd(uav_adrc_Y.ZTorque)), _mm_mul_pd(_mm_loadu_pd
      (&uav_adrc_ConstP.pooled12[i]), _mm_set1_pd(rtb_Sum1)))));
  }

  for (i = 2; i < 3; i++) {
    tmp_0[i] = ((uav_adrc_ConstP.pooled8[i + 3] * rtb_Gain2_h +
                 uav_adrc_ConstP.pooled8[i] * UnitDelay_DSTATE) +
                uav_adrc_ConstP.pooled8[i + 6] * UnitDelay_DSTATE_0) +
      (uav_adrc_ConstP.pooled11[i] * uav_adrc_Y.ZTorque +
       uav_adrc_ConstP.pooled12[i] * rtb_Sum1);
  }

  // Outport: '<Root>/Y Torque'
  uav_adrc_Y.YTorque = uav_adrc_Y.ZTorque;

  // Sum: '<S36>/Add1' incorporates:
  //   UnitDelay: '<S36>/Unit Delay'

  uav_adrc_DW.UnitDelay_DSTATE_f[0] = tmp_0[0];
  uav_adrc_DW.UnitDelay_DSTATE_f[1] = tmp_0[1];
  uav_adrc_DW.UnitDelay_DSTATE_f[2] = tmp_0[2];

  // Sum: '<S58>/Subtract' incorporates:
  //   Gain: '<S58>/Gain'
  //   Inport: '<Root>/Z Attitude State'
  //   UnitDelay: '<S58>/Unit Delay'

  rtb_Sum1 = uav_adrc_U.ZAttitudeState - ((0.0 * uav_adrc_DW.UnitDelay_DSTATE_h
    [1] + uav_adrc_DW.UnitDelay_DSTATE_h[0]) + 0.0 *
    uav_adrc_DW.UnitDelay_DSTATE_h[2]);

  // Sum: '<S57>/Sum1' incorporates:
  //   Gain: '<S57>/Gain'
  //   Gain: '<S57>/Gain2'
  //   Gain: '<S58>/Gain1'
  //   Inport: '<Root>/Z Attitude Ref'
  //   Sum: '<S57>/Sum'
  //   Sum: '<S57>/Sum2'
  //   Sum: '<S58>/Add'
  //   UnitDelay: '<S58>/Unit Delay'

  uav_adrc_Y.ZTorque = ((uav_adrc_U.ZAttitudeRef - (0.058235466415751391 *
    rtb_Sum1 + uav_adrc_DW.UnitDelay_DSTATE_h[0])) * 4.0 - (1.1646316877553027 *
    rtb_Sum1 + uav_adrc_DW.UnitDelay_DSTATE_h[1]) * 4.0) - (7.763952455012177 *
    rtb_Sum1 + uav_adrc_DW.UnitDelay_DSTATE_h[2]);
  if (tmp) {
    // Update for UnitDelay: '<S69>/Unit Delay'
    uav_adrc_DW.UnitDelay_DSTATE_am[0] = rtb_Gain4[0];
    uav_adrc_DW.UnitDelay_DSTATE_am[1] = rtb_Gain4[1];
    uav_adrc_DW.UnitDelay_DSTATE_am[2] = rtb_Gain4[2];
  }

  // UnitDelay: '<S58>/Unit Delay' incorporates:
  //   Gain: '<S58>/Gain2'

  rtb_Gain2_h = uav_adrc_DW.UnitDelay_DSTATE_h[1];
  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_h[0];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_h[2];

  // Sum: '<S58>/Add1' incorporates:
  //   Gain: '<S58>/Gain2'
  //   Gain: '<S58>/Gain3'
  //   Gain: '<S58>/Gain4'
  //   UnitDelay: '<S58>/Unit Delay'

  for (i = 0; i <= 0; i += 2) {
    _mm_storeu_pd(&tmp_0[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&uav_adrc_ConstP.pooled8[i + 3]), _mm_set1_pd(rtb_Gain2_h)),
      _mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled8[i]), _mm_set1_pd
                 (UnitDelay_DSTATE))), _mm_mul_pd(_mm_loadu_pd
      (&uav_adrc_ConstP.pooled8[i + 6]), _mm_set1_pd(UnitDelay_DSTATE_0))),
      _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&uav_adrc_ConstP.pooled11[i]),
      _mm_set1_pd(uav_adrc_Y.ZTorque)), _mm_mul_pd(_mm_loadu_pd
      (&uav_adrc_ConstP.pooled12[i]), _mm_set1_pd(rtb_Sum1)))));
  }

  for (i = 2; i < 3; i++) {
    tmp_0[i] = ((uav_adrc_ConstP.pooled8[i + 3] * rtb_Gain2_h +
                 uav_adrc_ConstP.pooled8[i] * UnitDelay_DSTATE) +
                uav_adrc_ConstP.pooled8[i + 6] * UnitDelay_DSTATE_0) +
      (uav_adrc_ConstP.pooled11[i] * uav_adrc_Y.ZTorque +
       uav_adrc_ConstP.pooled12[i] * rtb_Sum1);
  }

  // End of Sum: '<S58>/Add1'

  // Update for UnitDelay: '<S58>/Unit Delay'
  uav_adrc_DW.UnitDelay_DSTATE_h[0] = tmp_0[0];
  uav_adrc_DW.UnitDelay_DSTATE_h[1] = tmp_0[1];
  uav_adrc_DW.UnitDelay_DSTATE_h[2] = tmp_0[2];
  rate_scheduler((&uav_adrc_M));
}

// Model initialize function
void uav_adrc::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void uav_adrc::terminate()
{
  // (no terminate code required)
}

const char_T* uav_adrc::RT_MODEL_uav_adrc_T::getErrorStatus() const
{
  return (errorStatus);
}

void uav_adrc::RT_MODEL_uav_adrc_T::setErrorStatus(const char_T* const volatile
  aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
uav_adrc::uav_adrc() :
  uav_adrc_U(),
  uav_adrc_Y(),
  uav_adrc_DW(),
  uav_adrc_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
uav_adrc::~uav_adrc() = default;

// Real-Time Model get method
uav_adrc::RT_MODEL_uav_adrc_T * uav_adrc::getRTM()
{
  return (&uav_adrc_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
