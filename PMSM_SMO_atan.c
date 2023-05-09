/*
 * File: PMSM_SMO_atan.c
 *
 * Code generated for Simulink model 'PMSM_SMO_atan'.
 *
 * Model version                  : 1.283
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Tue May  9 14:55:37 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "PMSM_SMO_atan.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "PMSM_SMO_atan_private.h"
#include <float.h>

const real_T PMSM_SMO_atan_RGND = 0.0; /* real_T ground */

/* Continuous states */
X rtX;

/* Block signals and states (default storage) */
DW rtDW;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
real_T look1_binlx(real_T u0, const real_T bp0[], const real_T table[], uint32_T
                   maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 11;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  PMSM_SMO_atan_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  PMSM_SMO_atan_step();
  PMSM_SMO_atan_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  PMSM_SMO_atan_step();
  PMSM_SMO_atan_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T u1_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else {
    if (u1 < 0.0) {
      u1_0 = ceil(u1);
    } else {
      u1_0 = floor(u1);
    }

    if ((u1 != 0.0) && (u1 != u1_0)) {
      u1_0 = fabs(u0 / u1);
      if (!(fabs(u1_0 - floor(u1_0 + 0.5)) > DBL_EPSILON * u1_0)) {
        y = 0.0 * u0;
      } else {
        y = fmod(u0, u1);
      }
    } else {
      y = fmod(u0, u1);
    }
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void PMSM_SMO_atan_step(void)
{
  /* local block i/o variables */
  real_T rtb_Relay;
  real_T rtb_Relay1;
  real_T rtb_Relay2;
  real_T cosOut;
  real_T rtb_Add;
  real_T rtb_Add1;
  real_T rtb_Add2;
  real_T rtb_Add2_c;
  real_T rtb_Add2_e;
  real_T rtb_Add_b;
  real_T rtb_ElementaryMath_o1;
  real_T rtb_Filter;
  real_T rtb_Filter1;
  real_T rtb_u;
  real_T rtb_uFlux;
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    if (!(rtM->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&rtM->solverInfo, ((rtM->Timing.clockTickH0 + 1) *
        rtM->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&rtM->solverInfo, ((rtM->Timing.clockTick0 + 1) *
        rtM->Timing.stepSize0 + rtM->Timing.clockTickH0 * rtM->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  if (rtmIsMajorTimeStep(rtM)) {
    /* UnitDelay: '<S39>/Unit Delay' */
    rtDW.UnitDelay = rtDW.UnitDelay_DSTATE;

    /* DataTypeConversion: '<S39>/Data Type Conversion' incorporates:
     *  Logic: '<S39>/Logical Operator'
     */
    rtDW.DataTypeConversion = !(rtDW.UnitDelay != 0.0);

    /* UnitDelay: '<S39>/Unit Delay1' */
    rtDW.UnitDelay1 = rtDW.UnitDelay1_DSTATE;

    /* DataTypeConversion: '<S39>/Data Type Conversion1' incorporates:
     *  Logic: '<S39>/Logical Operator1'
     */
    rtDW.DataTypeConversion1 = !(rtDW.UnitDelay1 != 0.0);

    /* UnitDelay: '<S39>/Unit Delay2' */
    rtDW.UnitDelay2 = rtDW.UnitDelay2_DSTATE;

    /* DataTypeConversion: '<S39>/Data Type Conversion2' incorporates:
     *  Logic: '<S39>/Logical Operator2'
     */
    rtDW.DataTypeConversion2 = !(rtDW.UnitDelay2 != 0.0);
  }

  /* Trigonometry: '<S23>/Elementary Math' incorporates:
   *  Integrator: '<S21>/Int1'
   */
  cosOut = cos(rtX.Int1_CSTATE);
  rtb_ElementaryMath_o1 = sin(rtX.Int1_CSTATE);

  /* Fcn: '<S25>/Fcn' incorporates:
   *  Integrator: '<S27>/id '
   *  Integrator: '<S28>/iq '
   *  Trigonometry: '<S23>/Elementary Math'
   */
  rtDW.Fcn = rtX.iq_CSTATE * cosOut + rtX.id_CSTATE * rtb_ElementaryMath_o1;

  /* Fcn: '<S25>/Fcn1' incorporates:
   *  Integrator: '<S27>/id '
   *  Integrator: '<S28>/iq '
   *  Trigonometry: '<S23>/Elementary Math'
   */
  rtDW.Fcn1 = ((-rtX.iq_CSTATE - 1.7320508075688772 * rtX.id_CSTATE) * cosOut +
               (1.7320508075688772 * rtX.iq_CSTATE - rtX.id_CSTATE) *
               rtb_ElementaryMath_o1) * 0.5;

  /* S-Function (sfun_spssw_contc): '<S58>/State-Space' incorporates:
   *  Constant: '<S18>/DC'
   */

  /* Level2 S-Function Block: '<S58>/State-Space' (sfun_spssw_contc) */
  {
    SimStruct *rts = rtM->childSfunctions[0];
    sfcnOutputs(rts,0);
  }

  /* Sum: '<S31>/Sum2' incorporates:
   *  Fcn: '<S3>/Fcn'
   *  Integrator: '<S31>/Integrator'
   */
  rtb_Add2_e = rtX.Integrator_CSTATE - ((2.0 * rtDW.StateSpace_o1[11] -
    rtDW.StateSpace_o1[12]) - rtDW.StateSpace_o1[13]) / 3.0;

  /* Signum: '<S31>/Sign' */
  if (!rtIsNaN(rtb_Add2_e)) {
    if (rtb_Add2_e < 0.0) {
      rtb_Add2_e = -1.0;
    } else {
      rtb_Add2_e = (rtb_Add2_e > 0.0);
    }
  }

  /* End of Signum: '<S31>/Sign' */

  /* Gain: '<S31>/Ksw' */
  rtDW.Ksw = 200.0 * rtb_Add2_e;

  /* Sum: '<S31>/ ' incorporates:
   *  Fcn: '<S14>/Fcn'
   *  Gain: '<S31>/1//L'
   *  Gain: '<S31>/1//L '
   *  Gain: '<S31>/R//L'
   *  Integrator: '<S31>/Integrator'
   */
  rtDW.u = (((2.0 * rtDW.StateSpace_o1[8] - rtDW.StateSpace_o1[9]) -
             rtDW.StateSpace_o1[10]) / 3.0 * 117.64705882352941 -
            338.235294117647 * rtX.Integrator_CSTATE) - 117.64705882352941 *
    rtDW.Ksw;

  /* Sum: '<S31>/Sum5' incorporates:
   *  Fcn: '<S3>/Fcn1'
   *  Integrator: '<S31>/Integrator1'
   */
  rtb_Add2_e = rtX.Integrator1_CSTATE - (rtDW.StateSpace_o1[12] -
    rtDW.StateSpace_o1[13]) / 1.7320508075688772;

  /* Signum: '<S31>/Sign1' */
  if (!rtIsNaN(rtb_Add2_e)) {
    if (rtb_Add2_e < 0.0) {
      rtb_Add2_e = -1.0;
    } else {
      rtb_Add2_e = (rtb_Add2_e > 0.0);
    }
  }

  /* End of Signum: '<S31>/Sign1' */

  /* Gain: '<S31>/Ksw ' */
  rtDW.Ksw_i = 200.0 * rtb_Add2_e;

  /* Sum: '<S31>/   ' incorporates:
   *  Fcn: '<S14>/Fcn1'
   *  Gain: '<S31>/ 1//L'
   *  Gain: '<S31>/ 1//L '
   *  Gain: '<S31>/R//L '
   *  Integrator: '<S31>/Integrator1'
   */
  rtDW.u_j = ((rtDW.StateSpace_o1[9] - rtDW.StateSpace_o1[10]) /
              1.7320508075688772 * 117.64705882352941 - 338.235294117647 *
              rtX.Integrator1_CSTATE) - 117.64705882352941 * rtDW.Ksw_i;

  /* Lookup_n-D: '<S33>/Look-Up Table1' incorporates:
   *  Clock: '<S33>/Clock'
   *  Constant: '<S33>/Constant'
   *  Math: '<S33>/Math Function'
   */
  rtb_Add2_e = look1_binlx(rt_remd_snf(rtM->Timing.t[0], 0.0002),
    rtConstP.LookUpTable1_bp01Data, rtConstP.LookUpTable1_tableData, 2U);

  /* TransferFcn: '<S31>/Filter' */
  rtb_Filter = 20000.0 * rtX.Filter_CSTATE;

  /* TransferFcn: '<S31>/Filter1' */
  rtb_Filter1 = 20000.0 * rtX.Filter1_CSTATE;

  /* Fcn: '<S30>/Fcn' */
  rtb_uFlux = rt_powd_snf(rtb_Filter, 2.0) + rt_powd_snf(rtb_Filter1, 2.0);
  if (rtb_uFlux < 0.0) {
    rtb_uFlux = -sqrt(-rtb_uFlux);
  } else {
    rtb_uFlux = sqrt(rtb_uFlux);
  }

  /* End of Fcn: '<S30>/Fcn' */

  /* Gain: '<S30>/1//Flux' */
  rtb_uFlux *= 5.7142857142857144;

  /* Step: '<S32>/Step1' incorporates:
   *  Step: '<S56>/Step1'
   *  Step: '<S57>/Step1'
   */
  rtb_Add1 = rtM->Timing.t[0];

  /* Switch: '<S32>/Switch' incorporates:
   *  Constant: '<S30>/wc'
   *  Constant: '<S32>/Constant4'
   *  Fcn: '<S30>/Fcn1'
   *  Fcn: '<S30>/Fcn2'
   *  Step: '<S32>/Step1'
   *  Sum: '<S30>/  '
   */
  if (!(rtb_Add1 < 0.001)) {
    /* Fcn: '<S30>/Fcn1' */
    rtb_Add = fabs(rtb_Filter1);
    rtb_Filter = ((-rtb_Filter1 + rtb_Add) * 3.1415926535897931 / 2.0 / rtb_Add
                  + atan(-rtb_Filter / rtb_Filter1)) + atan(20000.0 / rtb_uFlux);
  } else {
    rtb_Filter = 0.0;
  }

  /* End of Switch: '<S32>/Switch' */

  /* Sum: '<Root>/  ' incorporates:
   *  Constant: '<Root>/Constant7'
   */
  rtb_u = rtb_Filter - 1.5707963267948966;

  /* Sum: '<Root>/Add1' incorporates:
   *  Constant: '<Root>/Constant6'
   *  Gain: '<Root>/Gain3'
   */
  rtb_Filter = 1000.0 - 2.3873241463784303 * rtb_uFlux;

  /* Switch: '<S57>/Switch' incorporates:
   *  Constant: '<S16>/Constant'
   *  Constant: '<S16>/Constant1'
   *  Constant: '<S57>/Constant4'
   *  Fcn: '<S16>/Fcn1'
   *  Step: '<S57>/Step1'
   *  Sum: '<S16>/Add'
   *  Sum: '<S16>/Add1'
   */
  if (!(rtb_Add1 < 0.001)) {
    rtb_uFlux = ((sin(rtb_u - 2.0943951023931953) * rtDW.StateSpace_o1[12] + sin
                  (rtb_u) * rtDW.StateSpace_o1[11]) + sin(rtb_u +
      2.0943951023931953) * rtDW.StateSpace_o1[13]) * -2.0 / 3.0;
  } else {
    rtb_uFlux = 0.0;
  }

  /* End of Switch: '<S57>/Switch' */

  /* Sum: '<S7>/Add' incorporates:
   *  Integrator: '<S7>/Integrator'
   */
  rtb_Filter1 = rtb_Filter + rtX.Integrator_CSTATE_c;

  /* Saturate: '<S7>/Saturation' */
  if (rtb_Filter1 > 21.0) {
    rtb_Filter1 = 21.0;
  } else if (rtb_Filter1 < -21.0) {
    rtb_Filter1 = -21.0;
  }

  /* End of Saturate: '<S7>/Saturation' */

  /* Sum: '<Root>/Add2' */
  rtb_Filter1 -= rtb_uFlux;

  /* Sum: '<S9>/Add' incorporates:
   *  Gain: '<S9>/Kp'
   *  Integrator: '<S9>/Integrator'
   */
  rtb_uFlux = 17.0 * rtb_Filter1 + rtX.Integrator_CSTATE_e;

  /* Saturate: '<S9>/Saturation' */
  if (rtb_uFlux > 350.0) {
    rtb_uFlux = 350.0;
  } else if (rtb_uFlux < -350.0) {
    rtb_uFlux = -350.0;
  }

  /* End of Saturate: '<S9>/Saturation' */

  /* Switch: '<S56>/Switch' incorporates:
   *  Constant: '<S16>/Constant'
   *  Constant: '<S16>/Constant1'
   *  Constant: '<S56>/Constant4'
   *  Fcn: '<S16>/Fcn'
   *  Step: '<S56>/Step1'
   *  Sum: '<S16>/Add'
   *  Sum: '<S16>/Add1'
   */
  if (!(rtb_Add1 < 0.001)) {
    rtb_Add = ((cos(rtb_u - 2.0943951023931953) * rtDW.StateSpace_o1[12] + cos
                (rtb_u) * rtDW.StateSpace_o1[11]) + cos(rtb_u +
                2.0943951023931953) * rtDW.StateSpace_o1[13]) * 2.0 / 3.0;
  } else {
    rtb_Add = 0.0;
  }

  /* End of Switch: '<S56>/Switch' */

  /* Sum: '<Root>/Add3' incorporates:
   *  Constant: '<Root>/Constant3'
   */
  rtb_Add = 0.0 - rtb_Add;

  /* Sum: '<S8>/Add' incorporates:
   *  Gain: '<S8>/Kp'
   *  Integrator: '<S8>/Integrator'
   */
  rtb_Add1 = 17.0 * rtb_Add + rtX.Integrator_CSTATE_ch;

  /* Saturate: '<S8>/Saturation' */
  if (rtb_Add1 > 350.0) {
    rtb_Add1 = 350.0;
  } else if (rtb_Add1 < -350.0) {
    rtb_Add1 = -350.0;
  }

  /* End of Saturate: '<S8>/Saturation' */

  /* Fcn: '<S2>/Fcn1' incorporates:
   *  Fcn: '<S2>/Fcn'
   */
  rtb_Add2 = cos(rtb_u);
  rtb_Add_b = sin(rtb_u);
  rtb_u = rtb_Add_b * rtb_Add1 + rtb_Add2 * rtb_uFlux;

  /* Fcn: '<S2>/Fcn' */
  rtb_uFlux = rtb_Add2 * rtb_Add1 - rtb_Add_b * rtb_uFlux;

  /* Gain: '<S34>/Gain' */
  rtb_Add1 = 1.7320508075688772 * rtb_uFlux;

  /* Sum: '<S34>/Add2' incorporates:
   *  Gain: '<S34>/Gain1'
   *  Gain: '<S34>/Gain2'
   *  Gain: '<S34>/Gain3'
   *  Sum: '<S34>/Add'
   *  Sum: '<S34>/Add1'
   *  Switch: '<S34>/Switch'
   *  Switch: '<S34>/Switch1'
   *  Switch: '<S34>/Switch2'
   */
  rtb_Add2 = (((rtb_Add1 - rtb_u > 0.0) << 1) + ((-rtb_Add1 - rtb_u > 0.0) << 2))
    + (rtb_u > 0.0);

  /* Gain: '<S35>/Gain' */
  rtb_u *= 1.732;

  /* Product: '<S35>/Divide' incorporates:
   *  Constant: '<S12>/T'
   *  Constant: '<S12>/Vdc'
   */
  rtb_Add1 = 0.0001 * rtb_u / 311.0;

  /* Gain: '<S35>/Gain2' */
  rtb_u *= 0.5;

  /* Gain: '<S35>/Gain1' */
  rtb_uFlux *= 1.5;

  /* MultiPortSwitch: '<S36>/Multiport Switch' incorporates:
   *  Constant: '<S12>/T'
   *  Constant: '<S12>/Vdc'
   *  Gain: '<S36>/Gain'
   *  Gain: '<S36>/Gain1'
   *  Gain: '<S36>/Gain2'
   *  Product: '<S35>/Divide1'
   *  Product: '<S35>/Divide2'
   *  Sum: '<S35>/Add'
   *  Sum: '<S35>/Add1'
   */
  switch ((int32_T)rtb_Add2) {
   case 1:
    rtb_Add_b = (rtb_u - rtb_uFlux) * 0.0001 / 311.0;

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S12>/T'
     *  Constant: '<S12>/Vdc'
     *  Product: '<S35>/Divide1'
     *  Product: '<S35>/Divide2'
     *  Sum: '<S35>/Add'
     *  Sum: '<S35>/Add1'
     */
    rtb_u = (rtb_u + rtb_uFlux) * 0.0001 / 311.0;
    break;

   case 2:
    rtb_Add_b = (rtb_u + rtb_uFlux) * 0.0001 / 311.0;

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S12>/T'
     *  Constant: '<S12>/Vdc'
     *  Gain: '<S36>/Gain1'
     *  Product: '<S35>/Divide1'
     *  Sum: '<S35>/Add'
     */
    rtb_u = -rtb_Add1;
    break;

   case 3:
    rtb_Add_b = -((rtb_u - rtb_uFlux) * 0.0001 / 311.0);

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S12>/T'
     *  Constant: '<S12>/Vdc'
     *  Gain: '<S36>/Gain'
     *  Product: '<S35>/Divide2'
     *  Sum: '<S35>/Add1'
     */
    rtb_u = rtb_Add1;
    break;

   case 4:
    rtb_Add_b = -rtb_Add1;

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S12>/T'
     *  Constant: '<S12>/Vdc'
     *  Gain: '<S36>/Gain1'
     *  Product: '<S35>/Divide2'
     *  Sum: '<S35>/Add1'
     */
    rtb_u = (rtb_u - rtb_uFlux) * 0.0001 / 311.0;
    break;

   case 5:
    rtb_Add_b = rtb_Add1;

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S12>/T'
     *  Constant: '<S12>/Vdc'
     *  Gain: '<S36>/Gain2'
     *  Product: '<S35>/Divide1'
     *  Sum: '<S35>/Add'
     */
    rtb_u = -((rtb_u + rtb_uFlux) * 0.0001 / 311.0);
    break;

   default:
    rtb_Add_b = -((rtb_u + rtb_uFlux) * 0.0001 / 311.0);

    /* MultiPortSwitch: '<S36>/Multiport Switch1' incorporates:
     *  Constant: '<S12>/T'
     *  Constant: '<S12>/Vdc'
     *  Gain: '<S36>/Gain'
     *  Gain: '<S36>/Gain2'
     *  Product: '<S35>/Divide1'
     *  Product: '<S35>/Divide2'
     *  Sum: '<S35>/Add'
     *  Sum: '<S35>/Add1'
     */
    rtb_u = -((rtb_u - rtb_uFlux) * 0.0001 / 311.0);
    break;
  }

  /* End of MultiPortSwitch: '<S36>/Multiport Switch' */

  /* Gain: '<S37>/Gain' incorporates:
   *  Constant: '<S12>/T'
   *  Sum: '<S37>/Add'
   */
  rtb_Add1 = ((0.0001 - rtb_Add_b) - rtb_u) * 0.25;

  /* Sum: '<S37>/Add1' incorporates:
   *  Gain: '<S37>/Gain1'
   */
  rtb_uFlux = 0.5 * rtb_Add_b + rtb_Add1;

  /* Sum: '<S37>/Add2' incorporates:
   *  Gain: '<S37>/Gain2'
   */
  rtb_Add2_c = 0.5 * rtb_u + rtb_uFlux;

  /* MultiPortSwitch: '<S38>/Multiport Switch' */
  switch ((int32_T)rtb_Add2) {
   case 1:
    rtb_Add_b = rtb_uFlux;
    break;

   case 2:
    rtb_Add_b = rtb_Add1;
    break;

   case 3:
    rtb_Add_b = rtb_Add1;
    break;

   case 4:
    rtb_Add_b = rtb_Add2_c;
    break;

   case 5:
    rtb_Add_b = rtb_Add2_c;
    break;

   default:
    rtb_Add_b = rtb_uFlux;
    break;
  }

  /* End of MultiPortSwitch: '<S38>/Multiport Switch' */

  /* Sum: '<S39>/Add' */
  rtb_Add_b = rtb_Add2_e - rtb_Add_b;

  /* Relay: '<S39>/Relay' */
  if (rtsiIsModeUpdateTimeStep(&rtM->solverInfo)) {
    rtDW.Relay_Mode = ((rtb_Add_b >= 2.2204460492503131E-16) || ((!(rtb_Add_b <=
      -2.2204460492503131E-16)) && rtDW.Relay_Mode));
  }

  /* Relay: '<S39>/Relay' */
  rtb_Relay = rtDW.Relay_Mode;

  /* MultiPortSwitch: '<S38>/Multiport Switch1' */
  switch ((int32_T)rtb_Add2) {
   case 1:
    rtb_u = rtb_Add1;
    break;

   case 2:
    rtb_u = rtb_Add2_c;
    break;

   case 3:
    rtb_u = rtb_uFlux;
    break;

   case 4:
    rtb_u = rtb_uFlux;
    break;

   case 5:
    rtb_u = rtb_Add1;
    break;

   default:
    rtb_u = rtb_Add2_c;
    break;
  }

  /* End of MultiPortSwitch: '<S38>/Multiport Switch1' */

  /* Sum: '<S39>/Add1' */
  rtb_u = rtb_Add2_e - rtb_u;

  /* Relay: '<S39>/Relay1' */
  if (rtsiIsModeUpdateTimeStep(&rtM->solverInfo)) {
    rtDW.Relay1_Mode = ((rtb_u >= 2.2204460492503131E-16) || ((!(rtb_u <=
      -2.2204460492503131E-16)) && rtDW.Relay1_Mode));
  }

  /* Relay: '<S39>/Relay1' */
  rtb_Relay1 = rtDW.Relay1_Mode;

  /* MultiPortSwitch: '<S38>/Multiport Switch2' */
  switch ((int32_T)rtb_Add2) {
   case 1:
    rtb_Add1 = rtb_Add2_c;
    break;

   case 2:
    rtb_Add1 = rtb_uFlux;
    break;

   case 3:
    rtb_Add1 = rtb_Add2_c;
    break;

   case 4:
    break;

   case 5:
    rtb_Add1 = rtb_uFlux;
    break;
  }

  /* End of MultiPortSwitch: '<S38>/Multiport Switch2' */

  /* Sum: '<S39>/Add2' */
  rtb_Add2_e -= rtb_Add1;

  /* Relay: '<S39>/Relay2' */
  if (rtsiIsModeUpdateTimeStep(&rtM->solverInfo)) {
    rtDW.Relay2_Mode = ((rtb_Add2_e >= 2.2204460492503131E-16) || ((!(rtb_Add2_e
      <= -2.2204460492503131E-16)) && rtDW.Relay2_Mode));
  }

  /* Relay: '<S39>/Relay2' */
  rtb_Relay2 = rtDW.Relay2_Mode;

  /* Gain: '<S8>/Ki' */
  rtDW.Ki = 5750.0 * rtb_Add;

  /* Gain: '<S9>/Ki' */
  rtDW.Ki_a = 5750.0 * rtb_Filter1;

  /* Gain: '<S7>/Ki' */
  rtDW.Ki_l = 0.2 * rtb_Filter;

  /* Gain: '<S21>/Gain' incorporates:
   *  Integrator: '<S21>/Int'
   */
  rtDW.Gain = 4.0 * rtX.Int_CSTATE;

  /* Fcn: '<S23>/Fcn2' incorporates:
   *  Fcn: '<S23>/Fcn3'
   */
  rtb_Add2_e = 2.0 * rtDW.StateSpace_o1[6] + rtDW.StateSpace_o1[7];

  /* Sum: '<S28>/Sum1' incorporates:
   *  Fcn: '<S23>/Fcn2'
   *  Gain: '<S28>/1//Lq'
   *  Gain: '<S28>/R//Lq'
   *  Gain: '<S28>/lam//Lq'
   *  Integrator: '<S27>/id '
   *  Integrator: '<S28>/iq '
   *  Product: '<S28>/Product1'
   *  Trigonometry: '<S23>/Elementary Math'
   */
  rtDW.Sum1 = (((1.7320508075688772 * rtDW.StateSpace_o1[7] *
                 rtb_ElementaryMath_o1 + rtb_Add2_e * cosOut) *
                0.33333333333333331 * 117.64705882352941 - 338.235294117647 *
                rtX.iq_CSTATE) - rtX.id_CSTATE * rtDW.Gain) - 20.588235294117645
    * rtDW.Gain;

  /* Sum: '<S27>/Sum' incorporates:
   *  Fcn: '<S23>/Fcn3'
   *  Gain: '<S27>/1//Ld'
   *  Gain: '<S27>/R//Ld'
   *  Integrator: '<S27>/id '
   *  Integrator: '<S28>/iq '
   *  Product: '<S27>/Product'
   *  Trigonometry: '<S23>/Elementary Math'
   */
  rtDW.Sum = ((-1.7320508075688772 * rtDW.StateSpace_o1[7] * cosOut + rtb_Add2_e
               * rtb_ElementaryMath_o1) * 0.33333333333333331 *
              117.64705882352941 - 338.235294117647 * rtX.id_CSTATE) + rtDW.Gain
    * rtX.iq_CSTATE;

  /* Signum: '<S29>/Sign' incorporates:
   *  Integrator: '<S21>/Int'
   */
  if (rtIsNaN(rtX.Int_CSTATE)) {
    rtb_Add1 = rtX.Int_CSTATE;
  } else if (rtX.Int_CSTATE < 0.0) {
    rtb_Add1 = -1.0;
  } else {
    rtb_Add1 = (rtX.Int_CSTATE > 0.0);
  }

  /* End of Signum: '<S29>/Sign' */

  /* Gain: '<S21>/Gain2' incorporates:
   *  Fcn: '<S19>/Te '
   *  Gain: '<S29>/Gain'
   *  Gain: '<S29>/Gain1'
   *  Integrator: '<S21>/Int'
   *  Integrator: '<S27>/id '
   *  Integrator: '<S28>/iq '
   *  Sum: '<S21>/Sum'
   *  Sum: '<S29>/Sum'
   */
  rtDW.Gain2 = ((0.0 * rtX.iq_CSTATE * rtX.id_CSTATE + 0.175 * rtX.iq_CSTATE) *
                6.0 - (0.0 * rtb_Add1 + 0.0 * rtX.Int_CSTATE)) * 1000.0;
  if (rtmIsMajorTimeStep(rtM)) {
    if (rtmIsMajorTimeStep(rtM)) {
      /* Update for UnitDelay: '<S39>/Unit Delay' */
      rtDW.UnitDelay_DSTATE = rtb_Relay;

      /* Update for UnitDelay: '<S39>/Unit Delay1' */
      rtDW.UnitDelay1_DSTATE = rtb_Relay1;

      /* Update for UnitDelay: '<S39>/Unit Delay2' */
      rtDW.UnitDelay2_DSTATE = rtb_Relay2;
    }

    /* Update for S-Function (sfun_spssw_contc): '<S58>/State-Space' incorporates:
     *  Constant: '<S18>/DC'
     */
    /* Level2 S-Function Block: '<S58>/State-Space' (sfun_spssw_contc) */
    {
      SimStruct *rts = rtM->childSfunctions[0];
      sfcnUpdate(rts,0);
      if (ssGetErrorStatus(rts) != (NULL))
        return;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++rtM->Timing.clockTick0)) {
      ++rtM->Timing.clockTickH0;
    }

    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [2.0E-7s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 2.0E-7, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      rtM->Timing.clockTick1++;
      if (!rtM->Timing.clockTick1) {
        rtM->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void PMSM_SMO_atan_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<S21>/Int1' */
  _rtXdot->Int1_CSTATE = rtDW.Gain;

  /* Derivatives for Integrator: '<S28>/iq ' */
  _rtXdot->iq_CSTATE = rtDW.Sum1;

  /* Derivatives for Integrator: '<S27>/id ' */
  _rtXdot->id_CSTATE = rtDW.Sum;

  /* Derivatives for Integrator: '<S31>/Integrator' */
  _rtXdot->Integrator_CSTATE = rtDW.u;

  /* Derivatives for Integrator: '<S31>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = rtDW.u_j;

  /* Derivatives for TransferFcn: '<S31>/Filter' */
  _rtXdot->Filter_CSTATE = -20000.0 * rtX.Filter_CSTATE;
  _rtXdot->Filter_CSTATE += rtDW.Ksw;

  /* Derivatives for TransferFcn: '<S31>/Filter1' */
  _rtXdot->Filter1_CSTATE = -20000.0 * rtX.Filter1_CSTATE;
  _rtXdot->Filter1_CSTATE += rtDW.Ksw_i;

  /* Derivatives for Integrator: '<S7>/Integrator' */
  _rtXdot->Integrator_CSTATE_c = rtDW.Ki_l;

  /* Derivatives for Integrator: '<S9>/Integrator' */
  _rtXdot->Integrator_CSTATE_e = rtDW.Ki_a;

  /* Derivatives for Integrator: '<S8>/Integrator' */
  _rtXdot->Integrator_CSTATE_ch = rtDW.Ki;

  /* Derivatives for Integrator: '<S21>/Int' */
  _rtXdot->Int_CSTATE = rtDW.Gain2;
}

/* Model initialize function */
void PMSM_SMO_atan_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->contStates = ((X *) &rtX);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetSolverName(&rtM->solverInfo,"ode3");
  rtM->solverInfoPtr = (&rtM->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = rtM->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;

    /* polyspace +2 MISRA2012:D4.1 [Justified:Low] "rtM points to
       static memory which is guaranteed to be non-NULL" */
    rtM->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    rtM->Timing.sampleTimes = (&rtM->Timing.sampleTimesArray[0]);
    rtM->Timing.offsetTimes = (&rtM->Timing.offsetTimesArray[0]);

    /* task periods */
    rtM->Timing.sampleTimes[0] = (0.0);
    rtM->Timing.sampleTimes[1] = (2.0E-7);

    /* task offsets */
    rtM->Timing.offsetTimes[0] = (0.0);
    rtM->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = rtM->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    rtM->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(rtM, -1);
  rtM->Timing.stepSize0 = 2.0E-7;
  rtM->solverInfoPtr = (&rtM->solverInfo);
  rtM->Timing.stepSize = (2.0E-7);
  rtsiSetFixedStepSize(&rtM->solverInfo, 2.0E-7);
  rtsiSetSolverMode(&rtM->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &rtM->NonInlinedSFcns.sfcnInfo;
    rtM->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(rtM)));
    rtM->Sizes.numSampTimes = (2);
    rtssSetNumRootSampTimesPtr(sfcnInfo, &rtM->Sizes.numSampTimes);
    rtM->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr(rtM)[0]);
    rtM->NonInlinedSFcns.taskTimePtrs[1] = &(rtmGetTPtr(rtM)[1]);
    rtssSetTPtrPtr(sfcnInfo,rtM->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(rtM));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(rtM));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput(rtM));
    rtssSetStepSizePtr(sfcnInfo, &rtM->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(rtM));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo, &rtM->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo, &rtM->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &rtM->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo, &rtM->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo, &rtM->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &rtM->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &rtM->solverInfoPtr);
  }

  rtM->Sizes.numSFcns = (1);

  /* register each child */
  {
    (void) memset((void *)&rtM->NonInlinedSFcns.childSFunctions[0], 0,
                  1*sizeof(SimStruct));
    rtM->childSfunctions = (&rtM->NonInlinedSFcns.childSFunctionPtrs[0]);
    rtM->childSfunctions[0] = (&rtM->NonInlinedSFcns.childSFunctions[0]);

    /* Level2 S-Function Block: PMSM_SMO_atan/<S58>/State-Space (sfun_spssw_contc) */
    {
      SimStruct *rts = rtM->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = rtM->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = rtM->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = rtM->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod, 0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset, 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &rtM->NonInlinedSFcns.blkInfo2[0]);
        ssSetBlkInfoSLSizePtr(rts, &rtM->NonInlinedSFcns.blkInfoSLSize[0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts, &rtM->
        NonInlinedSFcns.inputOutputPortInfo2[0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, rtM->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &rtM->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &rtM->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &rtM->NonInlinedSFcns.methods4[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &rtM->NonInlinedSFcns.statesInfo2[0]);
        ssSetPeriodicStatesInfo(rts, &rtM->NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 2);
        ssSetPortInfoForInputs(rts, &rtM->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &rtM->NonInlinedSFcns.Sfcn0.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts,
          &rtM->NonInlinedSFcns.Sfcn0.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &rtM->NonInlinedSFcns.Sfcn0.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &rtM->NonInlinedSFcns.Sfcn0.UPtrs0;

          {
            int_T i1;
            const real_T *u0;
            for (i1=0; i1 < 6; i1++) {
              sfcnUPtrs[i1] = ((const real_T*) &PMSM_SMO_atan_RGND);
            }

            sfcnUPtrs[6] = &rtDW.Fcn;
            sfcnUPtrs[7] = &rtDW.Fcn1;
            sfcnUPtrs[8] = &rtConstP.pooled2;
          }

          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 9);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &rtM->NonInlinedSFcns.Sfcn0.UPtrs1;
          sfcnUPtrs[0] = &rtDW.UnitDelay;
          sfcnUPtrs[1] = &rtDW.DataTypeConversion;
          sfcnUPtrs[2] = &rtDW.UnitDelay1;
          sfcnUPtrs[3] = &rtDW.DataTypeConversion1;
          sfcnUPtrs[4] = &rtDW.UnitDelay2;
          sfcnUPtrs[5] = &rtDW.DataTypeConversion2;
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidthAsInt(rts, 1, 6);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts, &rtM->NonInlinedSFcns.Sfcn0.outputPortInfo
          [0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->outputs =
          &rtM->NonInlinedSFcns.Sfcn0.outputPortInfoSLSize[0];
        _ssSetNumOutputPorts(rts, 2);
        _ssSetPortInfo2ForOutputUnits(rts,
          &rtM->NonInlinedSFcns.Sfcn0.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &rtM->NonInlinedSFcns.Sfcn0.outputPortCoSimAttribute[0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidthAsInt(rts, 0, 14);
          ssSetOutputPortSignal(rts, 0, ((real_T *) rtDW.StateSpace_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidthAsInt(rts, 1, 6);
          ssSetOutputPortSignal(rts, 1, ((real_T *) rtDW.StateSpace_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "State-Space");
      ssSetPath(rts, "PMSM_SMO_atan/powergui/EquivalentModel1/State-Space");
      ssSetRTModel(rts,rtM);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **) &rtM->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 12);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)rtConstInitP.StateSpace_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)rtConstInitP.StateSpace_P1_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)rtConstInitP.StateSpace_P1_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)rtConstP.StateSpace_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)rtConstInitP.StateSpace_P1_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)rtConstP.StateSpace_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)rtConstP.StateSpace_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)rtConstP.StateSpace_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)rtConstP.StateSpace_P9_Size);
        ssSetSFcnParam(rts, 9, (mxArray*)rtConstP.StateSpace_P9_Size);
        ssSetSFcnParam(rts, 10, (mxArray*)rtConstP.StateSpace_P6_Size);
        ssSetSFcnParam(rts, 11, (mxArray*)rtConstP.StateSpace_P12_Size);
      }

      /* work vectors */
      ssSetRWork(rts, (real_T *) &rtDW.StateSpace_RWORK);
      ssSetIWork(rts, (int_T *) &rtDW.StateSpace_IWORK[0]);
      ssSetPWork(rts, (void **) &rtDW.StateSpace_PWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &rtM->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &rtM->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 4);

        /* MODE */
        ssSetDWorkWidthAsInt(rts, 0, 7);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &rtDW.StateSpace_MODE[0]);

        /* RWORK */
        ssSetDWorkWidthAsInt(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_DOUBLE);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &rtDW.StateSpace_RWORK);

        /* IWORK */
        ssSetDWorkWidthAsInt(rts, 2, 9);
        ssSetDWorkDataType(rts, 2,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 2, 0);
        ssSetDWork(rts, 2, &rtDW.StateSpace_IWORK[0]);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 3, 19);
        ssSetDWorkDataType(rts, 3,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 3, 0);
        ssSetDWork(rts, 3, &rtDW.StateSpace_PWORK[0]);
      }

      ssSetModeVector(rts, (int_T *) &rtDW.StateSpace_MODE[0]);

      /* registration */
      sfun_spssw_contc(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.0);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
    }
  }

  /* Start for S-Function (sfun_spssw_contc): '<S58>/State-Space' incorporates:
   *  Constant: '<S18>/DC'
   */
  /* Level2 S-Function Block: '<S58>/State-Space' (sfun_spssw_contc) */
  {
    SimStruct *rts = rtM->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* InitializeConditions for Integrator: '<S21>/Int1' */
  rtX.Int1_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S28>/iq ' */
  rtX.iq_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S27>/id ' */
  rtX.id_CSTATE = 0.0;

  /* InitializeConditions for S-Function (sfun_spssw_contc): '<S58>/State-Space' incorporates:
   *  Constant: '<S18>/DC'
   */
  /* Level2 S-Function Block: '<S58>/State-Space' (sfun_spssw_contc) */
  {
    SimStruct *rts = rtM->childSfunctions[0];
    sfcnInitializeConditions(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* InitializeConditions for Integrator: '<S31>/Integrator' */
  rtX.Integrator_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S31>/Integrator1' */
  rtX.Integrator1_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S31>/Filter' */
  rtX.Filter_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S31>/Filter1' */
  rtX.Filter1_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S7>/Integrator' */
  rtX.Integrator_CSTATE_c = 0.0;

  /* InitializeConditions for Integrator: '<S9>/Integrator' */
  rtX.Integrator_CSTATE_e = 0.0;

  /* InitializeConditions for Integrator: '<S8>/Integrator' */
  rtX.Integrator_CSTATE_ch = 0.0;

  /* InitializeConditions for Integrator: '<S21>/Int' */
  rtX.Int_CSTATE = 0.0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
