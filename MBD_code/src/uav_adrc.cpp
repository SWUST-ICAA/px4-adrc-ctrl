//
// File: uav_adrc.cpp
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
#include "rtwtypes.h"

// Model step function
void uav_adrc::step()
{
  real_T tmp[3];
  real_T UnitDelay_DSTATE;
  real_T UnitDelay_DSTATE_0;
  real_T UnitDelay_DSTATE_1;
  real_T rtb_Gain1;
  real_T rtb_Subtract;
  int32_T i;

  // Sum: '<S25>/Subtract' incorporates:
  //   Gain: '<S25>/Gain'
  //   Inport: '<Root>/X Postion State'
  //   UnitDelay: '<S25>/Unit Delay'

  rtb_Gain1 = uav_adrc_U.XPostionState - ((0.0 * uav_adrc_DW.UnitDelay_DSTATE[1]
    + uav_adrc_DW.UnitDelay_DSTATE[0]) + 0.0 * uav_adrc_DW.UnitDelay_DSTATE[2]);

  // Sum: '<S24>/Sum1' incorporates:
  //   Gain: '<S24>/Gain'
  //   Gain: '<S24>/Gain2'
  //   Gain: '<S25>/Gain1'
  //   Inport: '<Root>/X Postion Ref '
  //   Sum: '<S24>/Sum'
  //   Sum: '<S24>/Sum2'
  //   Sum: '<S25>/Add'
  //   UnitDelay: '<S25>/Unit Delay'

  rtb_Subtract = ((uav_adrc_U.XPostionRef - (0.0089596212271162434 * rtb_Gain1 +
    uav_adrc_DW.UnitDelay_DSTATE[0])) * 2.25 - (0.026878823363131674 * rtb_Gain1
    + uav_adrc_DW.UnitDelay_DSTATE[1]) * 3.0) - (0.026878803204032076 *
    rtb_Gain1 + uav_adrc_DW.UnitDelay_DSTATE[2]);

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

  for (i = 0; i < 3; i++) {
    tmp[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
               uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
              uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
      (uav_adrc_ConstP.pooled7[i] * rtb_Subtract + uav_adrc_ConstP.pooled8[i] *
       rtb_Gain1);
  }

  // Outport: '<Root>/X Acceleration'
  uav_adrc_Y.XAcceleration = rtb_Subtract;

  // Update for UnitDelay: '<S25>/Unit Delay' incorporates:
  //   Sum: '<S25>/Add1'

  uav_adrc_DW.UnitDelay_DSTATE[0] = tmp[0];
  uav_adrc_DW.UnitDelay_DSTATE[1] = tmp[1];
  uav_adrc_DW.UnitDelay_DSTATE[2] = tmp[2];

  // Sum: '<S47>/Subtract' incorporates:
  //   Gain: '<S47>/Gain'
  //   Inport: '<Root>/Y Postion State'
  //   UnitDelay: '<S47>/Unit Delay'

  rtb_Subtract = uav_adrc_U.YPostionState - ((0.0 *
    uav_adrc_DW.UnitDelay_DSTATE_e[1] + uav_adrc_DW.UnitDelay_DSTATE_e[0]) + 0.0
    * uav_adrc_DW.UnitDelay_DSTATE_e[2]);

  // Sum: '<S46>/Sum1' incorporates:
  //   Gain: '<S46>/Gain'
  //   Gain: '<S46>/Gain2'
  //   Gain: '<S47>/Gain1'
  //   Inport: '<Root>/Y Postion Ref '
  //   Sum: '<S46>/Sum'
  //   Sum: '<S46>/Sum2'
  //   Sum: '<S47>/Add'
  //   UnitDelay: '<S47>/Unit Delay'

  rtb_Gain1 = ((uav_adrc_U.YPostionRef - (0.0089596212271162434 * rtb_Subtract +
    uav_adrc_DW.UnitDelay_DSTATE_e[0])) * 2.25 - (0.026878823363131674 *
    rtb_Subtract + uav_adrc_DW.UnitDelay_DSTATE_e[1]) * 3.0) -
    (0.026878803204032076 * rtb_Subtract + uav_adrc_DW.UnitDelay_DSTATE_e[2]);

  // UnitDelay: '<S47>/Unit Delay' incorporates:
  //   Gain: '<S47>/Gain2'

  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_e[1];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_e[0];
  UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_e[2];

  // Sum: '<S47>/Add1' incorporates:
  //   Gain: '<S47>/Gain2'
  //   Gain: '<S47>/Gain3'
  //   Gain: '<S47>/Gain4'
  //   UnitDelay: '<S47>/Unit Delay'

  for (i = 0; i < 3; i++) {
    tmp[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
               uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
              uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
      (uav_adrc_ConstP.pooled7[i] * rtb_Gain1 + uav_adrc_ConstP.pooled8[i] *
       rtb_Subtract);
  }

  // Outport: '<Root>/Y Acceleration'
  uav_adrc_Y.YAcceleration = rtb_Gain1;

  // Update for UnitDelay: '<S47>/Unit Delay' incorporates:
  //   Sum: '<S47>/Add1'

  uav_adrc_DW.UnitDelay_DSTATE_e[0] = tmp[0];
  uav_adrc_DW.UnitDelay_DSTATE_e[1] = tmp[1];
  uav_adrc_DW.UnitDelay_DSTATE_e[2] = tmp[2];

  // Sum: '<S69>/Subtract' incorporates:
  //   Gain: '<S69>/Gain'
  //   Inport: '<Root>/Z Postion State'
  //   UnitDelay: '<S69>/Unit Delay'

  rtb_Subtract = uav_adrc_U.ZPostionState - ((0.0 *
    uav_adrc_DW.UnitDelay_DSTATE_ey[1] + uav_adrc_DW.UnitDelay_DSTATE_ey[0]) +
    0.0 * uav_adrc_DW.UnitDelay_DSTATE_ey[2]);

  // Sum: '<S68>/Sum1' incorporates:
  //   Gain: '<S68>/Gain'
  //   Gain: '<S68>/Gain2'
  //   Gain: '<S69>/Gain1'
  //   Inport: '<Root>/Z Postion Ref '
  //   Sum: '<S68>/Sum'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S69>/Add'
  //   UnitDelay: '<S69>/Unit Delay'

  rtb_Gain1 = ((uav_adrc_U.ZPostionRef - (0.017838967641699233 * rtb_Subtract +
    uav_adrc_DW.UnitDelay_DSTATE_ey[0])) * 4.0 - (0.1070331636523693 *
    rtb_Subtract + uav_adrc_DW.UnitDelay_DSTATE_ey[1]) * 4.0) -
    (0.21406568510806817 * rtb_Subtract + uav_adrc_DW.UnitDelay_DSTATE_ey[2]);

  // UnitDelay: '<S69>/Unit Delay' incorporates:
  //   Gain: '<S69>/Gain2'

  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_ey[1];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_ey[0];
  UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_ey[2];

  // Sum: '<S69>/Add1' incorporates:
  //   Gain: '<S69>/Gain2'
  //   Gain: '<S69>/Gain3'
  //   Gain: '<S69>/Gain4'
  //   UnitDelay: '<S69>/Unit Delay'

  for (i = 0; i < 3; i++) {
    tmp[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
               uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
              uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
      (uav_adrc_ConstP.pooled7[i] * rtb_Gain1 + uav_adrc_ConstP.Gain4_Gain[i] *
       rtb_Subtract);
  }

  // Outport: '<Root>/Z Acceleration'
  uav_adrc_Y.ZAcceleration = rtb_Gain1;

  // Update for UnitDelay: '<S69>/Unit Delay' incorporates:
  //   Sum: '<S69>/Add1'

  uav_adrc_DW.UnitDelay_DSTATE_ey[0] = tmp[0];
  uav_adrc_DW.UnitDelay_DSTATE_ey[1] = tmp[1];
  uav_adrc_DW.UnitDelay_DSTATE_ey[2] = tmp[2];

  // Sum: '<S14>/Subtract' incorporates:
  //   Gain: '<S14>/Gain'
  //   Inport: '<Root>/q_ex'
  //   UnitDelay: '<S14>/Unit Delay'

  rtb_Subtract = uav_adrc_U.q_ex - ((0.0 * uav_adrc_DW.UnitDelay_DSTATE_n[1] +
    uav_adrc_DW.UnitDelay_DSTATE_n[0]) + 0.0 * uav_adrc_DW.UnitDelay_DSTATE_n[2]);

  // Gain: '<S13>/Gain1' incorporates:
  //   Constant: '<Root>/q_ex Ref'
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/Gain2'
  //   Gain: '<S14>/Gain1'
  //   Sum: '<S13>/Sum'
  //   Sum: '<S13>/Sum1'
  //   Sum: '<S13>/Sum2'
  //   Sum: '<S14>/Add'
  //   UnitDelay: '<S14>/Unit Delay'

  rtb_Gain1 = (((0.0 - (0.072256513671447253 * rtb_Subtract +
                        uav_adrc_DW.UnitDelay_DSTATE_n[0])) * 100.0 -
                (1.8062246992593298 * rtb_Subtract +
                 uav_adrc_DW.UnitDelay_DSTATE_n[1]) * 20.0) -
               (15.051088591129274 * rtb_Subtract +
                uav_adrc_DW.UnitDelay_DSTATE_n[2])) * 0.023255813953488372;

  // UnitDelay: '<S14>/Unit Delay' incorporates:
  //   Gain: '<S14>/Gain2'

  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_n[1];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_n[0];
  UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_n[2];

  // Sum: '<S14>/Add1' incorporates:
  //   Gain: '<S14>/Gain2'
  //   Gain: '<S14>/Gain3'
  //   Gain: '<S14>/Gain4'
  //   UnitDelay: '<S14>/Unit Delay'

  for (i = 0; i < 3; i++) {
    tmp[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
               uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
              uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
      (uav_adrc_ConstP.pooled14[i] * rtb_Gain1 + uav_adrc_ConstP.pooled15[i] *
       rtb_Subtract);
  }

  // Outport: '<Root>/BodyXTorque'
  uav_adrc_Y.BodyXTorque = rtb_Gain1;

  // Update for UnitDelay: '<S14>/Unit Delay' incorporates:
  //   Sum: '<S14>/Add1'

  uav_adrc_DW.UnitDelay_DSTATE_n[0] = tmp[0];
  uav_adrc_DW.UnitDelay_DSTATE_n[1] = tmp[1];
  uav_adrc_DW.UnitDelay_DSTATE_n[2] = tmp[2];

  // Sum: '<S36>/Subtract' incorporates:
  //   Gain: '<S36>/Gain'
  //   Inport: '<Root>/q_ey'
  //   UnitDelay: '<S36>/Unit Delay'

  rtb_Subtract = uav_adrc_U.q_ey - ((0.0 * uav_adrc_DW.UnitDelay_DSTATE_a[1] +
    uav_adrc_DW.UnitDelay_DSTATE_a[0]) + 0.0 * uav_adrc_DW.UnitDelay_DSTATE_a[2]);

  // Gain: '<S35>/Gain1' incorporates:
  //   Constant: '<Root>/q_ey Ref'
  //   Gain: '<S35>/Gain'
  //   Gain: '<S35>/Gain2'
  //   Gain: '<S36>/Gain1'
  //   Sum: '<S35>/Sum'
  //   Sum: '<S35>/Sum1'
  //   Sum: '<S35>/Sum2'
  //   Sum: '<S36>/Add'
  //   UnitDelay: '<S36>/Unit Delay'

  rtb_Gain1 = (((0.0 - (0.072256513671447253 * rtb_Subtract +
                        uav_adrc_DW.UnitDelay_DSTATE_a[0])) * 100.0 -
                (1.8062246992593298 * rtb_Subtract +
                 uav_adrc_DW.UnitDelay_DSTATE_a[1]) * 20.0) -
               (15.051088591129274 * rtb_Subtract +
                uav_adrc_DW.UnitDelay_DSTATE_a[2])) * 0.023255813953488372;

  // UnitDelay: '<S36>/Unit Delay' incorporates:
  //   Gain: '<S36>/Gain2'

  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_a[1];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_a[0];
  UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_a[2];

  // Sum: '<S36>/Add1' incorporates:
  //   Gain: '<S36>/Gain2'
  //   Gain: '<S36>/Gain3'
  //   Gain: '<S36>/Gain4'
  //   UnitDelay: '<S36>/Unit Delay'

  for (i = 0; i < 3; i++) {
    tmp[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
               uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
              uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
      (uav_adrc_ConstP.pooled14[i] * rtb_Gain1 + uav_adrc_ConstP.pooled15[i] *
       rtb_Subtract);
  }

  // Outport: '<Root>/BodyYTorque'
  uav_adrc_Y.BodyYTorque = rtb_Gain1;

  // Update for UnitDelay: '<S36>/Unit Delay' incorporates:
  //   Sum: '<S36>/Add1'

  uav_adrc_DW.UnitDelay_DSTATE_a[0] = tmp[0];
  uav_adrc_DW.UnitDelay_DSTATE_a[1] = tmp[1];
  uav_adrc_DW.UnitDelay_DSTATE_a[2] = tmp[2];

  // Sum: '<S58>/Subtract' incorporates:
  //   Gain: '<S58>/Gain'
  //   Inport: '<Root>/q_ez'
  //   UnitDelay: '<S58>/Unit Delay'

  rtb_Subtract = uav_adrc_U.q_ez - ((0.0 * uav_adrc_DW.UnitDelay_DSTATE_g[1] +
    uav_adrc_DW.UnitDelay_DSTATE_g[0]) + 0.0 * uav_adrc_DW.UnitDelay_DSTATE_g[2]);

  // Gain: '<S57>/Gain1' incorporates:
  //   Constant: '<Root>/q_ez Ref'
  //   Gain: '<S57>/Gain2'
  //   Gain: '<S58>/Gain1'
  //   Sum: '<S57>/Sum'
  //   Sum: '<S57>/Sum1'
  //   Sum: '<S57>/Sum2'
  //   Sum: '<S58>/Add'
  //   UnitDelay: '<S58>/Unit Delay'

  rtb_Gain1 = (((0.0 - (0.0089596212271162434 * rtb_Subtract +
                        uav_adrc_DW.UnitDelay_DSTATE_g[0])) -
                (0.026878823363131674 * rtb_Subtract +
                 uav_adrc_DW.UnitDelay_DSTATE_g[1]) * 2.0) -
               (0.026878803204032076 * rtb_Subtract +
                uav_adrc_DW.UnitDelay_DSTATE_g[2])) * 0.04;

  // Outport: '<Root>/BodyZTorque'
  uav_adrc_Y.BodyZTorque = rtb_Gain1;

  // UnitDelay: '<S58>/Unit Delay' incorporates:
  //   Gain: '<S58>/Gain2'

  UnitDelay_DSTATE = uav_adrc_DW.UnitDelay_DSTATE_g[1];
  UnitDelay_DSTATE_0 = uav_adrc_DW.UnitDelay_DSTATE_g[0];
  UnitDelay_DSTATE_1 = uav_adrc_DW.UnitDelay_DSTATE_g[2];

  // Sum: '<S58>/Add1' incorporates:
  //   Gain: '<S58>/Gain2'
  //   Gain: '<S58>/Gain3'
  //   Gain: '<S58>/Gain4'
  //   UnitDelay: '<S58>/Unit Delay'

  for (i = 0; i < 3; i++) {
    tmp[i] = ((uav_adrc_ConstP.pooled2[i + 3] * UnitDelay_DSTATE +
               uav_adrc_ConstP.pooled2[i] * UnitDelay_DSTATE_0) +
              uav_adrc_ConstP.pooled2[i + 6] * UnitDelay_DSTATE_1) +
      (uav_adrc_ConstP.Gain3_Gain[i] * rtb_Gain1 + uav_adrc_ConstP.pooled8[i] *
       rtb_Subtract);
  }

  // End of Sum: '<S58>/Add1'

  // Update for UnitDelay: '<S58>/Unit Delay'
  uav_adrc_DW.UnitDelay_DSTATE_g[0] = tmp[0];
  uav_adrc_DW.UnitDelay_DSTATE_g[1] = tmp[1];
  uav_adrc_DW.UnitDelay_DSTATE_g[2] = tmp[2];
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
