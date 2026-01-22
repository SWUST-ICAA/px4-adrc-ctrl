//
// File: uav_adrc.h
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
#ifndef uav_adrc_h_
#define uav_adrc_h_
#include <cmath>
#include "rtwtypes.h"
#include "uav_adrc_types.h"

// Class declaration for model uav_adrc
class uav_adrc final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_uav_adrc_T {
    real_T UnitDelay_DSTATE[3];        // '<S21>/Unit Delay'
    real_T UnitDelay_DSTATE_a[3];      // '<S10>/Unit Delay'
  };

  // Constant parameters (default storage)
  struct ConstP_uav_adrc_T {
    // Expression: ad
    //  Referenced by: '<S21>/Gain2'

    real_T Gain2_Gain[9];

    // Pooled Parameter (Expression: c)
    //  Referenced by:
    //    '<S10>/Gain'
    //    '<S21>/Gain'

    real_T pooled2[3];

    // Expression: ad
    //  Referenced by: '<S10>/Gain2'

    real_T Gain2_Gain_bn[9];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_uav_adrc_T {
    real_T PostionRef;                 // '<Root>/Postion Ref'
    real_T PositionState;              // '<Root>/Position State'
    real_T AttitudeRef;                // '<Root>/Attitude Ref'
    real_T AttitudeState;              // '<Root>/Attitude State'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_uav_adrc_T {
    real_T Acceleration;               // '<Root>/Acceleration'
    real_T Torque;                     // '<Root>/Torque'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_uav_adrc_T {
    const char_T * volatile errorStatus;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      struct {
        uint8_T TID[3];
      } TaskCounters;
    } Timing;

    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  uav_adrc(uav_adrc const&) = delete;

  // Assignment Operator
  uav_adrc& operator= (uav_adrc const&) & = delete;

  // Move Constructor
  uav_adrc(uav_adrc &&) = delete;

  // Move Assignment Operator
  uav_adrc& operator= (uav_adrc &&) = delete;

  // Real-Time Model get method
  uav_adrc::RT_MODEL_uav_adrc_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_uav_adrc_T *pExtU_uav_adrc_T)
  {
    uav_adrc_U = *pExtU_uav_adrc_T;
  }

  // Root outports get method
  const ExtY_uav_adrc_T &getExternalOutputs() const
  {
    return uav_adrc_Y;
  }

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  uav_adrc();

  // Destructor
  ~uav_adrc();

  // private data and function members
 private:
  // External inputs
  ExtU_uav_adrc_T uav_adrc_U;

  // External outputs
  ExtY_uav_adrc_T uav_adrc_Y;

  // Block states
  DW_uav_adrc_T uav_adrc_DW;

  // Real-Time Model
  RT_MODEL_uav_adrc_T uav_adrc_M;
};

// Constant parameters (default storage)
extern const uav_adrc::ConstP_uav_adrc_T uav_adrc_ConstP;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S9>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S12>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S13>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S20>/Gain' : Eliminated nontunable gain of 1
//  Block '<S20>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S23>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S24>/Zero-Order Hold' : Eliminated since input and output rates are identical


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'uav_adrc'
//  '<S1>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1'
//  '<S2>'   : 'uav_adrc/Postion Active Disturbance Rejection Control'
//  '<S3>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC'
//  '<S4>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/error feedback'
//  '<S5>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/extended state observer'
//  '<S6>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/input saturation'
//  '<S7>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/rZOH'
//  '<S8>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/yZOH'
//  '<S9>'   : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/error feedback/seccond order'
//  '<S10>'  : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/extended state observer/discrete time'
//  '<S11>'  : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/input saturation/passthrough'
//  '<S12>'  : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/rZOH/enabled'
//  '<S13>'  : 'uav_adrc/Attitude Active Disturbance Rejection Control1/ADRC/yZOH/enabled'
//  '<S14>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC'
//  '<S15>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S16>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S17>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S18>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S19>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S20>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S21>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S22>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S23>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S24>'  : 'uav_adrc/Postion Active Disturbance Rejection Control/ADRC/yZOH/enabled'

#endif                                 // uav_adrc_h_

//
// File trailer for generated code.
//
// [EOF]
//
