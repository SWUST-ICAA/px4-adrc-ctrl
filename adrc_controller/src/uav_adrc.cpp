//
// File: uav_adrc.cpp
//
// Code generated for Simulink model 'uav_adrc'.
//
// Model version                  : 1.2
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Jan 22 10:38:17 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
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
  if ((uav_adrc_M->Timing.TaskCounters.TID[1]) > 9) {// Sample time: [0.001s, 0.0s] 
    uav_adrc_M->Timing.TaskCounters.TID[1] = 0;
  }

  (uav_adrc_M->Timing.TaskCounters.TID[2])++;
  if ((uav_adrc_M->Timing.TaskCounters.TID[2]) > 99) {// Sample time: [0.01s, 0.0s] 
    uav_adrc_M->Timing.TaskCounters.TID[2] = 0;
  }
}

// Model step function
void uav_adrc::step()
{
  __m128d tmp;
  __m128d tmp_0;
  real_T rtb_Gain2[3];
  real_T rtb_Subtract;
  int32_T i;
  if ((&uav_adrc_M)->Timing.TaskCounters.TID[2] == 0) {
    // Gain: '<S21>/Gain'
    rtb_Subtract = 0.0;
    for (i = 0; i < 3; i++) {
      // Gain: '<S21>/Gain2' incorporates:
      //   UnitDelay: '<S21>/Unit Delay'

      rtb_Gain2[i] = (uav_adrc_ConstP.Gain2_Gain[i + 3] *
                      uav_adrc_DW.UnitDelay_DSTATE[1] +
                      uav_adrc_ConstP.Gain2_Gain[i] *
                      uav_adrc_DW.UnitDelay_DSTATE[0]) +
        uav_adrc_ConstP.Gain2_Gain[i + 6] * uav_adrc_DW.UnitDelay_DSTATE[2];

      // Gain: '<S21>/Gain' incorporates:
      //   UnitDelay: '<S21>/Unit Delay'

      rtb_Subtract += uav_adrc_ConstP.pooled2[i] *
        uav_adrc_DW.UnitDelay_DSTATE[i];
    }

    // Sum: '<S21>/Subtract' incorporates:
    //   Gain: '<S21>/Gain'
    //   Inport: '<Root>/Position State'

    rtb_Subtract = uav_adrc_U.PositionState - rtb_Subtract;

    // Gain: '<S21>/Gain1' incorporates:
    //   Sum: '<S21>/Add'
    //   UnitDelay: '<S21>/Unit Delay'

    tmp_0 = _mm_set1_pd(rtb_Subtract);
    tmp = _mm_add_pd(_mm_mul_pd(_mm_set_pd(2.5875074351664682,
      0.25918177931828223), tmp_0), _mm_loadu_pd(&uav_adrc_DW.UnitDelay_DSTATE[0]));

    // Gain: '<S21>/Gain3' incorporates:
    //   Gain: '<S21>/Gain1'
    //   Sum: '<S21>/Add'

    _mm_storeu_pd(&uav_adrc_DW.UnitDelay_DSTATE[0], tmp);

    // Sum: '<S21>/Add' incorporates:
    //   Gain: '<S21>/Gain1'
    //   Gain: '<S21>/Gain3'
    //   UnitDelay: '<S21>/Unit Delay'

    uav_adrc_DW.UnitDelay_DSTATE[2] += 8.61784444348992 * rtb_Subtract;

    // Sum: '<S20>/Sum1' incorporates:
    //   Gain: '<S20>/Gain2'
    //   Inport: '<Root>/Postion Ref'
    //   Sum: '<S20>/Sum'
    //   Sum: '<S20>/Sum2'

    uav_adrc_Y.Acceleration = ((uav_adrc_U.PostionRef -
      uav_adrc_DW.UnitDelay_DSTATE[0]) - 2.0 * uav_adrc_DW.UnitDelay_DSTATE[1])
      - uav_adrc_DW.UnitDelay_DSTATE[2];

    // Gain: '<S21>/Gain3' incorporates:
    //   Gain: '<S21>/Gain4'
    //   Sum: '<S21>/Add1'

    tmp_0 = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(0.01, 5.0E-5),
      _mm_set1_pd(uav_adrc_Y.Acceleration)), _mm_mul_pd(_mm_set_pd
      (2.6736858796013676, 0.28548774589212145), tmp_0)), _mm_loadu_pd
                       (&rtb_Gain2[0]));

    // Gain: '<S21>/Gain3' incorporates:
    //   Sum: '<S21>/Add1'
    //   UnitDelay: '<S21>/Unit Delay'

    _mm_storeu_pd(&uav_adrc_DW.UnitDelay_DSTATE[0], tmp_0);

    // Sum: '<S21>/Add1' incorporates:
    //   Gain: '<S21>/Gain3'
    //   Gain: '<S21>/Gain4'
    //   UnitDelay: '<S21>/Unit Delay'

    uav_adrc_DW.UnitDelay_DSTATE[2] = (0.0 * uav_adrc_Y.Acceleration +
      8.61784444348992 * rtb_Subtract) + rtb_Gain2[2];
  }

  if ((&uav_adrc_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S10>/Gain'
    rtb_Subtract = 0.0;
    for (i = 0; i < 3; i++) {
      // Gain: '<S10>/Gain2' incorporates:
      //   UnitDelay: '<S10>/Unit Delay'

      rtb_Gain2[i] = (uav_adrc_ConstP.Gain2_Gain_bn[i + 3] *
                      uav_adrc_DW.UnitDelay_DSTATE_a[1] +
                      uav_adrc_ConstP.Gain2_Gain_bn[i] *
                      uav_adrc_DW.UnitDelay_DSTATE_a[0]) +
        uav_adrc_ConstP.Gain2_Gain_bn[i + 6] * uav_adrc_DW.UnitDelay_DSTATE_a[2];

      // Gain: '<S10>/Gain' incorporates:
      //   UnitDelay: '<S10>/Unit Delay'

      rtb_Subtract += uav_adrc_ConstP.pooled2[i] *
        uav_adrc_DW.UnitDelay_DSTATE_a[i];
    }

    // Sum: '<S10>/Subtract' incorporates:
    //   Gain: '<S10>/Gain'
    //   Inport: '<Root>/Attitude State'

    rtb_Subtract = uav_adrc_U.AttitudeState - rtb_Subtract;

    // Gain: '<S10>/Gain1' incorporates:
    //   Sum: '<S10>/Add'
    //   UnitDelay: '<S10>/Unit Delay'

    tmp_0 = _mm_set1_pd(rtb_Subtract);
    tmp = _mm_add_pd(_mm_mul_pd(_mm_set_pd(1.1646316877553027,
      0.058235466415751391), tmp_0), _mm_loadu_pd
                     (&uav_adrc_DW.UnitDelay_DSTATE_a[0]));

    // Gain: '<S10>/Gain3' incorporates:
    //   Gain: '<S10>/Gain1'
    //   Sum: '<S10>/Add'

    _mm_storeu_pd(&uav_adrc_DW.UnitDelay_DSTATE_a[0], tmp);

    // Sum: '<S10>/Add' incorporates:
    //   Gain: '<S10>/Gain1'
    //   Gain: '<S10>/Gain3'
    //   UnitDelay: '<S10>/Unit Delay'

    uav_adrc_DW.UnitDelay_DSTATE_a[2] += 7.763952455012177 * rtb_Subtract;

    // Sum: '<S9>/Sum1' incorporates:
    //   Gain: '<S9>/Gain'
    //   Gain: '<S9>/Gain2'
    //   Inport: '<Root>/Attitude Ref'
    //   Sum: '<S9>/Sum'
    //   Sum: '<S9>/Sum2'

    uav_adrc_Y.Torque = ((uav_adrc_U.AttitudeRef -
                          uav_adrc_DW.UnitDelay_DSTATE_a[0]) * 4.0 - 4.0 *
                         uav_adrc_DW.UnitDelay_DSTATE_a[1]) -
      uav_adrc_DW.UnitDelay_DSTATE_a[2];

    // Gain: '<S10>/Gain3' incorporates:
    //   Gain: '<S10>/Gain4'
    //   Sum: '<S10>/Add1'

    tmp_0 = _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set_pd(0.001, 5.0E-7),
      _mm_set1_pd(uav_adrc_Y.Torque)), _mm_mul_pd(_mm_set_pd(1.1723956402103148,
      0.0594039800797342), tmp_0)), _mm_loadu_pd(&rtb_Gain2[0]));

    // Gain: '<S10>/Gain3' incorporates:
    //   Sum: '<S10>/Add1'
    //   UnitDelay: '<S10>/Unit Delay'

    _mm_storeu_pd(&uav_adrc_DW.UnitDelay_DSTATE_a[0], tmp_0);

    // Sum: '<S10>/Add1' incorporates:
    //   Gain: '<S10>/Gain3'
    //   Gain: '<S10>/Gain4'
    //   UnitDelay: '<S10>/Unit Delay'

    uav_adrc_DW.UnitDelay_DSTATE_a[2] = (0.0 * uav_adrc_Y.Torque +
      7.763952455012177 * rtb_Subtract) + rtb_Gain2[2];
  }

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
