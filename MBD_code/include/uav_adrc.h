//
// File: uav_adrc.h
//
// Code generated for Simulink model 'uav_adrc'.
//
// Model version                  : 1.34
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Fri Jan 30 12:50:42 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
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
    real_T UnitDelay_DSTATE[3];        // '<S25>/Unit Delay'
    real_T UnitDelay_DSTATE_e[3];      // '<S47>/Unit Delay'
    real_T UnitDelay_DSTATE_ey[3];     // '<S69>/Unit Delay'
    real_T UnitDelay_DSTATE_n[3];      // '<S14>/Unit Delay'
    real_T UnitDelay_DSTATE_a[3];      // '<S36>/Unit Delay'
    real_T UnitDelay_DSTATE_g[3];      // '<S58>/Unit Delay'
  };

  // Constant parameters (default storage)
  struct ConstP_uav_adrc_T {
    // Pooled Parameter (Expression: ad)
    //  Referenced by:
    //    '<S14>/Gain2'
    //    '<S25>/Gain2'
    //    '<S36>/Gain2'
    //    '<S47>/Gain2'
    //    '<S58>/Gain2'
    //    '<S69>/Gain2'

    real_T pooled2[9];

    // Pooled Parameter (Expression: bd)
    //  Referenced by:
    //    '<S25>/Gain3'
    //    '<S47>/Gain3'
    //    '<S69>/Gain3'

    real_T pooled6[3];

    // Pooled Parameter (Expression: ad*ld)
    //  Referenced by:
    //    '<S25>/Gain4'
    //    '<S47>/Gain4'
    //    '<S69>/Gain4'

    real_T pooled7[3];

    // Pooled Parameter (Expression: bd)
    //  Referenced by:
    //    '<S14>/Gain3'
    //    '<S36>/Gain3'

    real_T pooled13[3];

    // Pooled Parameter (Expression: ad*ld)
    //  Referenced by:
    //    '<S14>/Gain4'
    //    '<S36>/Gain4'

    real_T pooled14[3];

    // Expression: bd
    //  Referenced by: '<S58>/Gain3'

    real_T Gain3_Gain[3];

    // Expression: ad*ld
    //  Referenced by: '<S58>/Gain4'

    real_T Gain4_Gain[3];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_uav_adrc_T {
    real_T XPostionRef;                // '<Root>/X Postion Ref '
    real_T YPostionRef;                // '<Root>/Y Postion Ref '
    real_T ZPostionRef;                // '<Root>/Z Postion Ref '
    real_T XPostionState;              // '<Root>/X Postion State'
    real_T YPostionState;              // '<Root>/Y Postion State'
    real_T ZPostionState;              // '<Root>/Z Postion State'
    real_T q_ex;                       // '<Root>/q_ex'
    real_T q_ey;                       // '<Root>/q_ey'
    real_T q_ez;                       // '<Root>/q_ez'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_uav_adrc_T {
    real_T XAcceleration;              // '<Root>/X Acceleration'
    real_T YAcceleration;              // '<Root>/Y Acceleration'
    real_T ZAcceleration;              // '<Root>/Z Acceleration'
    real_T BodyXTorque;                // '<Root>/BodyXTorque'
    real_T BodyYTorque;                // '<Root>/BodyYTorque'
    real_T BodyZTorque;                // '<Root>/BodyZTorque'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_uav_adrc_T {
    const char_T * volatile errorStatus;
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
//  Block '<S16>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S17>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S24>/Gain' : Eliminated nontunable gain of 1
//  Block '<S24>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S27>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S28>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S38>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S39>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S46>/Gain' : Eliminated nontunable gain of 1
//  Block '<S46>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S49>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S50>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S60>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S61>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S68>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S71>/Zero-Order Hold' : Eliminated since input and output rates are identical
//  Block '<S72>/Zero-Order Hold' : Eliminated since input and output rates are identical


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
//  '<S1>'   : 'uav_adrc/X Attitude Active Disturbance Rejection Control'
//  '<S2>'   : 'uav_adrc/X Postion Active Disturbance Rejection Control'
//  '<S3>'   : 'uav_adrc/Y Attitude Active Disturbance Rejection Control'
//  '<S4>'   : 'uav_adrc/Y Postion Active Disturbance Rejection Control'
//  '<S5>'   : 'uav_adrc/Z Attitude Active Disturbance Rejection Control'
//  '<S6>'   : 'uav_adrc/Z Postion Active Disturbance Rejection Control'
//  '<S7>'   : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC'
//  '<S8>'   : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S9>'   : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S10>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S11>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S12>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S13>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S14>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S15>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S16>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S17>'  : 'uav_adrc/X Attitude Active Disturbance Rejection Control/ADRC/yZOH/enabled'
//  '<S18>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC'
//  '<S19>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S20>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S21>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S22>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S23>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S24>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S25>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S26>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S27>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S28>'  : 'uav_adrc/X Postion Active Disturbance Rejection Control/ADRC/yZOH/enabled'
//  '<S29>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC'
//  '<S30>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S31>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S32>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S33>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S34>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S35>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S36>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S37>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S38>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S39>'  : 'uav_adrc/Y Attitude Active Disturbance Rejection Control/ADRC/yZOH/enabled'
//  '<S40>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC'
//  '<S41>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S42>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S43>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S44>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S45>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S46>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S47>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S48>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S49>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S50>'  : 'uav_adrc/Y Postion Active Disturbance Rejection Control/ADRC/yZOH/enabled'
//  '<S51>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC'
//  '<S52>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S53>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S54>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S55>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S56>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S57>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S58>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S59>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S60>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S61>'  : 'uav_adrc/Z Attitude Active Disturbance Rejection Control/ADRC/yZOH/enabled'
//  '<S62>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC'
//  '<S63>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/error feedback'
//  '<S64>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/extended state observer'
//  '<S65>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/input saturation'
//  '<S66>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/rZOH'
//  '<S67>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/yZOH'
//  '<S68>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/error feedback/seccond order'
//  '<S69>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/extended state observer/discrete time'
//  '<S70>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/input saturation/passthrough'
//  '<S71>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/rZOH/enabled'
//  '<S72>'  : 'uav_adrc/Z Postion Active Disturbance Rejection Control/ADRC/yZOH/enabled'

#endif                                 // uav_adrc_h_

//
// File trailer for generated code.
//
// [EOF]
//
