//#############################################################################
//
// FILE:   satindest.h
//
// TITLE:  C28x InstaSPIN Proportional-Integral (PI) controller library
//         (floating point)
//
//#############################################################################
// $TI Release: MotorControl SDK v4.00.00.00 $
// $Release Date: Thu Feb 17 18:05:18 CST 2022 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef SATINDEST_H
#define SATINDEST_H

/*************************************************************************************************************************************************************************************/
// If building with a C++ compiler, make all of the definitions in this header have a C binding.
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/*************************************************************************************************************************************************************************************/

//#include "types.h"
//#include "libraries/math/include/math.h"
//#include <math.h>

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "math_types.h"



#define LDQ_TEST_NUMBER (4)         // Parameter: Number. Inductance LUT size.

#define LdqEstStep_START                          (1)
#define LdqEstStep_NEGATIVE_CURRENT_ESTIMATION    (2)
#define LdqEstStep_POSITIVE_CURRENT_ESTIMATION    (3)
#define LdqEstStep_FINISH                         (4)
#define LdqEstStep_OPERATING_POINT_CHANGE         (5)

#define sgn_POSITIVE (+1)
#define sgn_NEGATIVE (-1)

#define D_AXIS (1)
#define Q_AXIS (2)

#define MtrType_INDUCTION               (1)
#define MtrType_IPMSM                   (2)
#define MtrType_SYNRM                   (3)

/*************************************************************************************************************************************************************************************/
// Defines the SATINDEST object
/*************************************************************************************************************************************************************************************/
typedef struct _SATINDEST_Obj_
{

    float32_t VariableInit;                 // Data: TRUE/FALSE. Estimation variables initialization enable.
    float32_t ExecuteEnable;                // Data: TRUE/FALSE. Parameter calculations are enabled.
    uint16_t InjSineEnable;                   // Data: TRUE/FALSE. Sinuoidal current injection is enabled.
    int16_t CurrentSign;                    // Data: Number. Positive=(+1), Negative=(-1).
    uint16_t Axis;                          // Data: Number. D_AXIS=1, Q_AXIS=2.
    uint16_t OpPtNo;                        // Data: Number. Magnetic operating point number.
    uint16_t EstStep;                       // Data: Number. Step number in the estimation routine for that operating point.
    uint32_t CtrlCycleCounter;                // Data: Number. Counter for control cycle entries here or execution of this MACRO.
    uint32_t InjSampleCounter;                // Data: Number. Counter for injected sample numbers of sinusoidal injection current.
    uint32_t InjSinePrdCounter;               // Data: Number. Counter for completed periods of injected sinusoidal current.
    uint32_t InjCurHalfPrdSampleNo;           // Data: Number. Half of sample number of injected sinusoidal current in its one period.
    uint32_t dcCurHalfPrdSampleNo;            // Data: Number. Half of sample number of square-wave current to determine magnetic operating point and keep rotor stationary.

    float32_t InjCurAngle;                  // Data: Per-Unit. Injected AC current angle to be used in 'sine' function.
    float32_t InjCurAngleStep;              // Data: Per-Unit. Injected AC current angle step.
    float32_t InputGainDFT;                 // Data: Gain. The coefficient used to multiply the DFT input to prevent overflow and to find exact amplitude of the frequency component.
    float32_t InjCurFreq;                   // Data: Per-Unit. Injected current frequency, actual values after 'InjCurHalfPrdSampleNo' selection.
    float32_t VmagnDFT;                     // Data: Per-Unit. Voltage peak magnitude of the fundamental frequency component.
    float32_t VphaseDFT;                    // Data: Per-Unit. Voltage phase angle of the fundamental frequency component.
    float32_t ImagnDFT;                     // Data: Per-Unit. Current peak magnitude of the fundamental frequency component.
    float32_t IphaseDFT;                    // Data: Per-Unit. Current phase angle of the fundamental frequency component.
    float32_t CosDFT;                       // Data: Gain. Cosine value used in DFT computation.
    float32_t SinDFT;                       // Data: Gain. Sine value used in DFT computation.
    float32_t VrealDFT;                     // Data: Per-Unit. Real part of the DFT of the voltage.
    float32_t VimagDFT;                     // Data: Per-Unit. Imaginary part of the DFT of the voltage.
    float32_t IrealDFT;                     // Data: Per-Unit. Real part of the DFT of the current.
    float32_t IimagDFT;                     // Data: Per-Unit. Imaginary part of the DFT of the current.
    float32_t Lest;                         // Data: Per-Unit. Calculated 'differential inductance' at a given magnetic operating point.

    uint16_t MotorType;                       // Parameter: Number. IM=1, IPMSM=2, SynRM=3.
    uint32_t Nbase;                           // Parameter: Number. Base sample size: Fsampling/Fbase.
    uint32_t InjSettleWait_PrdNo;             // Parameter: Number. Number of injected sinusoidal current periods to be completed for settling.
    uint32_t dcSettleWait_SampleNo;           // Parameter: Number. Number of sample number to be completed for DC current settling after reference changes.
    float32_t InjCurFreqInit;               // Parameter: Per-Unit. Injected sinusoidal current frequency, initialization values.
    float32_t InjCurMagn;                   // Parameter: Per-Unit. Injected sinusoidal current amplitude.
    float32_t IrefDCStep;                   // Parameter: Per-Unit. Amplitude of DC current (=square wave amplitude) change size.

    float32_t Ifbk;                         // Input: Per-Unit. Related axis current feedback.
    float32_t Vref;                         // Input: Per-Unit. Related axis voltage reference.

    float32_t Finish;                       // Output: TRUE/FALSE. Identification is finished.
    float32_t Iref;                         // Output: Per-Unit. Related axis current reference.
    float32_t Lsigma [LDQ_TEST_NUMBER+1];   // Output: Per-Unit. Measured leakage inductance with varying current, (Induction motor).
    float32_t Ld[2*LDQ_TEST_NUMBER+1];      // Output: Per-Unit. Measured D-axis inductance with varying Id, Ld (Id), (Synchronous motor).
    float32_t Lq[2*LDQ_TEST_NUMBER+1];      // Output: Per-Unit. Measured Q-axis inductance with varying Iq, Lq (Iq), (Synchronous motor).

    float32_t Vdc_Base;
    float32_t Vph_Base;
    float32_t I_Base;
    float32_t F_Base;


} SATINDEST_Obj;

/*****************************************************************************/
//  Defines the SATINDEST handle
/*****************************************************************************/

typedef struct _SATINDEST_Obj_ *SATINDEST_Handle;

/*****************************************************************************/

extern SATINDEST_Handle SATINDEST_init(void *pMemory, const size_t numBytes);

extern void SATINDEST_setParams(SATINDEST_Handle handle);

/*****************************************************************************/

// fonksiyonun inputlarini girmeyi unutma

static inline void SATINDEST_run(SATINDEST_Handle handle, const float32_t Vd_fb, const float32_t Vq_fb, const float32_t id_fb, const float32_t iq_fb)
{

    SATINDEST_Obj *obj = (SATINDEST_Obj *)handle;


        /* Initialize some variables. */
        /* This enables safe start and usage of this routine again without resetting CPU. */


    if (obj -> VariableInit == 1.0f)
           {  obj -> VariableInit = 0.0f;
              obj -> ExecuteEnable = 0.0f;
              obj -> InjSineEnable = 0;
              obj -> OpPtNo = 0;
              obj -> Axis = D_AXIS; /* Initialize. */
              obj -> EstStep = LdqEstStep_POSITIVE_CURRENT_ESTIMATION; /* To shorthen the test. As Idc is zero at the beginning (OpPtNo is initialized to zero), no need for neg current.*/

              obj -> CtrlCycleCounter = 0;
              obj -> InjSampleCounter = 0;
              obj -> InjSinePrdCounter = 0;


              obj -> InjCurHalfPrdSampleNo = (obj -> Nbase * (1 / obj -> InjCurFreqInit)) / (2); /* Here, '_IQdiv' won't overflow because InjCurFreq > 1 p.u. mostly. */

              if ( (((obj -> Nbase * (1.0f / obj -> InjCurFreqInit)) / 2.0f)  - obj -> InjCurHalfPrdSampleNo) >= 0.5f ) /* Round the float variable into integer. Useful only in FLOAT_MATH */
              {  obj -> InjCurHalfPrdSampleNo++;}


              /* Calculate the count number for half period of square wave current: DC settling count + (AC settling prd + DFT period)*AC Period count */
              obj -> dcCurHalfPrdSampleNo = obj -> dcSettleWait_SampleNo + (obj -> InjSettleWait_PrdNo + 1) * (2 * obj -> InjCurHalfPrdSampleNo);

              /* Notice that '_IQdiv' works for division of normal intergers as well as '_iq' format. */
              obj -> InjCurAngleStep = 1.0f / (2.0f * obj -> InjCurHalfPrdSampleNo);

              /* We can update inj. freq. after selecting 'InjCurHalfPrdSampleNo' because exact freq. is now determined with 'InjCurHalfPrdSampleNo'. */
              obj -> InjCurFreq = obj -> InjCurAngleStep / (1.0f / obj -> Nbase); /* 1/Nbase = Fbase/Fs */

              obj -> InjCurAngle = 0.0f;
              obj -> VrealDFT = 0.0f;
              obj -> VimagDFT = 0.0f;
              obj -> IrealDFT = 0.0f;
              obj -> IimagDFT = 0.0f;

              /* 'InputGainDFT' is to prevent overflow in DFT memories. Normally we need to divide by (N/2). (1UL<<(30-GLOBAL_Q)) is to use all 32-bit capacity. */
              obj ->InputGainDFT = 2.0f * obj -> InjCurAngleStep;

              obj -> Finish = 0.0f;
           }

    // Provide inputs  // I added this part into the SATINDEST_run function
           if ( (obj -> MotorType == MtrType_INDUCTION) || ( ((obj -> MotorType == MtrType_IPMSM) || (obj -> MotorType == MtrType_SYNRM)) && (obj -> Axis == D_AXIS) ) )
           {
               obj -> Ifbk = id_fb / obj -> I_Base;
               obj -> Vref = Vd_fb / obj -> Vph_Base;
            }

            if ( ((obj -> MotorType == MtrType_IPMSM) || (obj -> MotorType == MtrType_SYNRM)) && (obj -> Axis == Q_AXIS) )
            {
                obj -> Ifbk = iq_fb / obj -> I_Base; ;
                obj -> Ifbk = Vq_fb / obj -> Vph_Base ;
             }

    /* DIRECT DISCRETE FOURIER TRANSFORM. */
       /* 'v.InputGainDFT' is to prevent overflow in the memories. */
       if (obj -> InjSinePrdCounter == obj -> InjSettleWait_PrdNo)
          {  // obj.SinDFT = _IQsinPU(v.InjCurAngle);
             // obj.CosDFT = _IQcosPU(v.InjCurAngle);

           obj -> SinDFT = sinf(obj -> InjCurAngle * MATH_TWO_PI);
           obj -> CosDFT = cosf(obj -> InjCurAngle * MATH_TWO_PI);

           obj -> VrealDFT += (obj -> Vref * obj -> CosDFT * obj -> InputGainDFT);
           obj -> VimagDFT -= (obj -> Vref * obj -> SinDFT * obj -> InputGainDFT);
           obj -> IrealDFT += (obj -> Ifbk * obj -> CosDFT * obj -> InputGainDFT);
           obj -> IimagDFT -= (obj -> Ifbk * obj -> SinDFT * obj -> InputGainDFT);

          }


       /* INJECTION PERIOD HANDLER: Enables signal detection by DFT and inductance calculation. */
       /* In this ctrl cycle, the last sample of the injected current period  is being applied. */
       if ( obj -> InjSampleCounter == (2 * obj -> InjCurHalfPrdSampleNo) )
          {
           obj -> InjSampleCounter = 0;
           obj -> InjSinePrdCounter++;
             /* Just after the injection period for DFT finishes, enable execution. */
             if (obj -> InjSinePrdCounter == (obj -> InjSettleWait_PrdNo + 1))
                {  obj -> InjSinePrdCounter = 0;
                   obj -> ExecuteEnable = 1.0f;}
          }


       /* PARAMETER CALCULATIONS */
       if (obj -> ExecuteEnable == 1.0f)
          {  obj -> ExecuteEnable = 0.0f;

             /* SIGNAL AMPLITUDE & PHASE CALCULATION. */
            // v.VphaseDFT = _IQatan2PU(v.VimagDFT, v.VrealDFT) - _IQdiv2(v.InjCurAngleStep); /* Half-sample ZOH delay is taken into account. */
            // v.IphaseDFT = _IQatan2PU(v.IimagDFT, v.IrealDFT);
            // v.VmagnDFT = _IQsqrt( _IQmpy(v.VrealDFT, v.VrealDFT) + _IQmpy(v.VimagDFT, v.VimagDFT) );
            // v.ImagnDFT = _IQsqrt( _IQmpy(v.IrealDFT, v.IrealDFT) + _IQmpy(v.IimagDFT, v.IimagDFT) );

          obj -> VphaseDFT = atan2f(obj -> VimagDFT, obj -> VrealDFT) - (obj -> InjCurAngleStep / 2.0f); /* Half-sample ZOH delay is taken into account. */
          obj -> IphaseDFT = atan2f(obj -> IimagDFT, obj -> IrealDFT);
          obj -> VmagnDFT = sqrtf( (obj -> VrealDFT * obj -> VrealDFT) + (obj -> VimagDFT * obj -> VimagDFT));
          obj -> ImagnDFT = sqrtf( (obj -> IrealDFT * obj -> IrealDFT) + (obj -> IimagDFT * obj -> IimagDFT));

             /* Reset DFT variables for re-use in the next operating point. */
          obj -> VrealDFT = 0.0f;
          obj -> VimagDFT = 0.0f;
          obj -> IrealDFT = 0.0f;
          obj -> IimagDFT = 0.0f;

          obj -> Lest = ((obj -> VmagnDFT / (obj -> ImagnDFT * obj -> InjCurFreq)) * sinf(obj -> VphaseDFT - obj -> IphaseDFT) );
             if (obj -> MotorType == MtrType_INDUCTION)
                {  obj -> Lsigma[obj -> OpPtNo] = obj -> Lest;}
             else if ( (obj -> MotorType == MtrType_IPMSM) || (obj -> MotorType == MtrType_SYNRM) )
                { if (obj -> Axis == D_AXIS)
                     {  obj -> Ld[LDQ_TEST_NUMBER + obj -> CurrentSign*(int16_t)obj -> OpPtNo] = obj -> Lest;} /* Be careful with multiplication. Use type-casting because CurrentSign is 'int16' and OpPtNo 'Uint16'. */
                  else
                     {  obj -> Lq[LDQ_TEST_NUMBER + obj -> CurrentSign*(int16_t)obj -> OpPtNo] = obj -> Lest;} /* Be careful with multiplication. Use type-casting because CurrentSign is 'int16' and OpPtNo 'Uint16'. */
                }
          }

       /* ESTIMATION STEPS */
       obj -> CtrlCycleCounter++;
       if (obj -> EstStep == LdqEstStep_START)
          {  obj ->CurrentSign = sgn_POSITIVE;
             if (obj -> CtrlCycleCounter == (obj -> dcCurHalfPrdSampleNo>>1) )
                {  obj -> CtrlCycleCounter = 0;
                   obj -> EstStep = LdqEstStep_NEGATIVE_CURRENT_ESTIMATION;}
          }

       else if (obj -> EstStep == LdqEstStep_NEGATIVE_CURRENT_ESTIMATION)
          {  obj -> CurrentSign = sgn_NEGATIVE;
             if (obj -> CtrlCycleCounter >= obj -> dcSettleWait_SampleNo)
                {  obj -> InjSineEnable = 1;}
             if (obj -> CtrlCycleCounter == obj -> dcCurHalfPrdSampleNo)
                {  obj -> CtrlCycleCounter = 0;
                   obj ->InjSineEnable = 1;
                   obj ->EstStep = LdqEstStep_POSITIVE_CURRENT_ESTIMATION;}
          }

       else if (obj -> EstStep == LdqEstStep_POSITIVE_CURRENT_ESTIMATION)
          {  obj -> CurrentSign = sgn_POSITIVE;
             if (obj -> CtrlCycleCounter >= obj -> dcSettleWait_SampleNo)
                {  obj -> InjSineEnable = 1;}
             if (obj -> CtrlCycleCounter == obj -> dcCurHalfPrdSampleNo)
                {  obj -> CtrlCycleCounter = 0;
                   obj -> InjSineEnable = 0;
                   if (obj -> MotorType == MtrType_INDUCTION)
                      {  obj -> EstStep = LdqEstStep_OPERATING_POINT_CHANGE;}
                   else if ( (obj -> MotorType == MtrType_IPMSM) || (obj -> MotorType == MtrType_SYNRM) )
                      {  obj -> EstStep = LdqEstStep_FINISH;}
                }
          }

       else if (obj -> EstStep == LdqEstStep_FINISH)
          {  obj -> CurrentSign = sgn_NEGATIVE;
             if (obj -> CtrlCycleCounter >= obj -> dcCurHalfPrdSampleNo)
                {  obj -> CurrentSign = sgn_POSITIVE;}
             if (obj -> CtrlCycleCounter == (obj -> dcCurHalfPrdSampleNo + (obj -> dcCurHalfPrdSampleNo>>1)) )
                {  obj -> CtrlCycleCounter = 0;
                   obj -> EstStep = LdqEstStep_OPERATING_POINT_CHANGE;}
          }

       /* Operating point change. */
       if (obj -> EstStep == LdqEstStep_OPERATING_POINT_CHANGE)
          {
             if (obj -> MotorType == MtrType_INDUCTION)
                {  obj -> EstStep = LdqEstStep_POSITIVE_CURRENT_ESTIMATION;}
             else if ( (obj -> MotorType == MtrType_IPMSM) || (obj -> MotorType == MtrType_SYNRM) )
                {  obj -> EstStep = LdqEstStep_START;}

             /* STOP */
             obj -> OpPtNo++;
             if (obj -> OpPtNo > LDQ_TEST_NUMBER)
                {  if (obj -> MotorType == MtrType_INDUCTION)
                      {  obj -> Finish = 1.0f;
                         obj -> VariableInit = 1.0f;}
                   else if ( (obj -> MotorType == MtrType_IPMSM) || (obj -> MotorType == MtrType_SYNRM) )
                      {  if (obj -> Axis == D_AXIS)
                            {  obj -> Axis = Q_AXIS;
                               obj -> EstStep = LdqEstStep_POSITIVE_CURRENT_ESTIMATION;
                               obj -> OpPtNo = 0;}
                         else
                            {  obj -> Finish = 1.0f;
                               obj ->VariableInit = 1.0;}
                       }
                }
          }


       /* ASSIGN REFERENCE CURRENT FOR SMALL SIGNAL ANALYSIS AROUND AN OPERATING POINT. */
       /* Compute the angle. Saturate the angle rate within (0, 1).  */
       if (obj -> InjSineEnable == 1)
          {  obj -> InjSampleCounter++;}
       else
          {  obj -> InjSampleCounter = 0;}

       obj -> InjCurAngle = obj -> InjSampleCounter * obj -> InjCurAngleStep;
       while (obj -> InjCurAngle > 1.0f)
             {  obj -> InjCurAngle -= 1.0f;}
       while (obj -> InjCurAngle < 0.0f)
             {  obj -> InjCurAngle += 1.0f;}

       /* Be careful with multiplication between 'CurrentSign' and 'OpPtNo'. Use type-casting (int16) for OpPtNo because CurrentSign is 'int16' and OpPtNo is 'Uint16'. */
       obj -> Iref = ( (obj -> CurrentSign*(int16_t)obj ->OpPtNo) * obj -> IrefDCStep) + ( obj -> InjCurMagn * sinf(obj -> InjCurAngle * MATH_TWO_PI));
       /* Increase ac amplitude the zero dc case */
       if (obj -> OpPtNo == 0) {  obj -> Iref = ( (LDQ_TEST_NUMBER * (obj -> IrefDCStep / 2.0f)) * sinf(obj -> InjCurAngle * MATH_TWO_PI));}


    return;
} // end of PI_run_series() function

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
//extern void
//SELFCOMM_setParams(SELFCOMM_Handle handle);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of PI_H defines
