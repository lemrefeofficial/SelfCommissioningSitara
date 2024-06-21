//#############################################################################
//
// FILE:   fluxlinest.h
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

#ifndef FLUXLINEST_H
#define FLUXLINEST_H

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

#define MtrType_INDUCTION               (1)
#define MtrType_IPMSM                   (2)
#define MtrType_SYNRM                   (3)

/*************************************************************************************************************************************************************************************/
// Defines the FLUXLINEST object
/*************************************************************************************************************************************************************************************/
typedef struct _FLUXLINEST_Obj_
{

    float32_t VariableInit;                // Data: TRUE/FALSE. Variable initialization is enabled.
    float32_t TimeOutError;                // Data: TRUE/FALSE. Error. 'SpeedFbk' could not stabilize to 'SpeedRef' in 'MaxTime4Settle' counts.
    float32_t SpeedRampArrives;            // Data: TRUE/FALSE. Ramped speed reference almost arrives at the actual speed reference.
    float32_t MagnFluxEstEn;               // Data: TRUE/FALSE. Magnet flux estimation is enabled.
    float32_t SlowDownEn;                  // Data: TRUE/FALSE. Motor is slowed down to zero after estimation step is finished.
    float32_t CtrlCycleCounter;            // Data: Number. Counter for control cycle entries here or execution of this MACRO.
    float32_t AveragerInputGain;        // Data: Gain. A coefficient used in the averager input to prevent overflow and to find exact amplitude of the frequency component.
    float32_t SpeedErrFiltered;         // Data: Per-Unit. Filtered error between 'SpeedRef' and 'SpeedFbk'. It is used to understand whether the speed is stabilized.
    float32_t IqFbkDerivative;          // Data: Per-Unit. Derivative of IqFbk. Low-pass filtered to limit noise amplificaton. Used to include inductive effect in back-emf calculation.
    float32_t IqFbkPast;                // Data: Per-Unit. One sample delayed 'IqFbk'.
    float32_t SpeedFbk_Average;         // Data: Per-Unit. 'SpeedFbk' average.
    float32_t BackEMF_Average;          // Data: Per-Unit. 'BackEMF' average.

    uint32_t AveragingSize;             // Parameter: Number. Sample size to be averaged in the test. Stopping condition for the identification.
    float32_t WaitingTime2Settle_Speed; // Parameter: Number. Samples to wait for speed loop to settle.
    float32_t MaxTime4Settle;           // Parameter: Number. Max aAllowed time for settling before beginning estimation.
    float32_t TestSpeed;                // Parameter: Per-Unit. Amplitude of test speed.
    float32_t SpeedErrLim4Settle;       // Parameter: Per-Unit. Speed error threshold to check whether the speed is almost stabilized at the reference.
    float32_t Rs;                       // Parameter: Per-Unit. Stator resistance.
    float32_t Lq;                       // Parameter: Per-Unit. Q-axis inductance.
    float32_t TimePU;                   // Parameter: Per-Unit. Per-Unit value of sampling time.
    float32_t LPF_IqDeriv_Gain;         // Parameter: Per-Unit. Multiplier of correction term in 'IqFbkDerivative' calculation.
    float32_t LPF_SpeedErr_Gain;        // Parameter: Per-Unit. Multiplier of correction term in 'SpeedErrFiltered' calculation.

    float32_t SpeedFbk;                 // Input: Per-Unit. Speed feedback.
    float32_t IqFbk;                    // Input: Per-Unit. Q-axis current feedback.
    float32_t VqRef;                    // Input: Per-Unit. Q-axis voltage reference.

    float32_t SpeedRef;                 // Output: Per-Unit. Speed reference.
    float32_t MagnFlux;                 // Output: Per-Unit. Measured permanent-magnet flux linkage.
    float32_t Finish;                   // Output: TRUE/FALSE. Identification is finished.

    float32_t Vdc_Base;
    float32_t Vph_Base;
    float32_t I_Base;
    float32_t F_Base;

    float32_t BASE_FREQ;


    float32_t test1;


} FLUXLINEST_Obj;

/*****************************************************************************/
//  Defines the FLUXLINEST handle
/*****************************************************************************/

typedef struct _FLUXLINEST_Obj_ *FLUXLINEST_Handle;

/*****************************************************************************/

extern FLUXLINEST_Handle FLUXLINEST_init(void *pMemory, const size_t numBytes);

extern void FLUXLINEST_setParams(FLUXLINEST_Handle handle);

/*****************************************************************************/

static inline void FLUXLINEST_run(FLUXLINEST_Handle handle, const float32_t MechSpeed, const float32_t iq_fb, const float32_t Vq_fb)
{

    FLUXLINEST_Obj *obj = (FLUXLINEST_Obj *)handle;

        /* Initialize some variables. */
        /* This enables safe start and usage of this routine again without resetting CPU. */

    if (obj -> VariableInit == 1.0f)
       {  obj ->VariableInit = 0.0f;

          obj -> TimeOutError = 0.0f;
          obj -> SpeedRampArrives = 0.0f;
          obj -> MagnFluxEstEn = 0.0f;
          obj -> SlowDownEn = 0.0f;
          obj -> CtrlCycleCounter = 0.0f;

          /* Find '1/N' or '1/AveragingSize'. They will be used as a multiplier for input signal in DFT so that DFT will not overflow if input is always less than _IQ(1.0). */

          // obj -> AveragingSize = _IQsat(obj -> AveragingSize, (1UL<<29), 1UL); /* Safety check for over/underflow-free correct working of below code. */
          obj -> AveragingSize = MATH_sat(obj -> AveragingSize, (1UL<<29), 1UL); /* Safety check for over/underflow-free correct working of below code. */

          /* Notice that '_IQdiv' works for division of normal intergers as well as '_iq' format. */
          /* Use full capacity of 32-bit. Enlarge the input gain of the DFT or avergaer not to loose information for trancation. But then at the end, divide the result. */
          obj -> AveragerInputGain = ( (float)(1UL<<(30-24)) / (float)(obj -> AveragingSize) );

          /* Assign the reference test speed */
          obj -> SpeedRef = obj -> TestSpeed;
          obj -> SpeedErrFiltered = obj -> TestSpeed; /* Precaution for pre-mature pass from STEP-1 to STEP-2 due to the if-comparison in STEP-1. */

          obj -> IqFbkDerivative = 0.0f;
          obj -> IqFbkPast = 0.0f;

          obj -> SpeedFbk_Average = 0.0f;
          obj -> BackEMF_Average = 0.0f;

          obj -> Finish = 0.0f;
       }

  obj -> SpeedFbk =  MechSpeed / obj -> F_Base;
  obj -> IqFbk = iq_fb / obj -> I_Base;
  obj -> VqRef = Vq_fb / obj -> Vph_Base;

    /* Speed control error is filtered by a 1st order Low-Pass Filter (Backward Euler discretized) because the error will be used in comparison, noise may cause wrong decision. */
    // obj -> SpeedErrFiltered += _IQmpy(obj -> LPF_SpeedErr_Gain, ((obj -> SpeedRef - obj -> SpeedFbk) - obj -> SpeedErrFiltered));
    obj -> SpeedErrFiltered += (obj -> LPF_SpeedErr_Gain * ((obj -> SpeedRef - obj -> SpeedFbk) - obj -> SpeedErrFiltered));

    /* STEP-1 : Wait until motor speed stabilizes to the test speed */
    if ((obj -> SpeedRampArrives == 1.0f) && (obj -> MagnFluxEstEn == 0.0f) && (obj -> SlowDownEn == 0.0f))
       {  obj -> CtrlCycleCounter++;
          if (obj -> CtrlCycleCounter >= obj -> WaitingTime2Settle_Speed)
             {  /* Option-1 : Only time-based decision making. */
                /* obj -> MagnFluxEstEn = 1.0f; */
                /* obj -> CtrlCycleCounter = 0.0f; */

                /* Option-2 : Checking Speed error vanishes as a second parameter on top of the 'WaitingTime2Settle_Speed' timer for decision making. */
                if (MATH_abs(obj -> SpeedErrFiltered) < obj -> SpeedErrLim4Settle)
                  {  obj -> MagnFluxEstEn = 1.0f;
                     obj -> CtrlCycleCounter = 0.0f;}
                /* Part of Option-2. If Id cannot settle in a maximum allowed settling time, then 'TimeOutError' is set TRUE. */
                if (obj -> CtrlCycleCounter >= obj -> MaxTime4Settle)
                   {  obj -> TimeOutError = 1.0f;
                      obj -> MagnFluxEstEn = 1.0f;
                      obj -> CtrlCycleCounter = 0.0f;}
             }
       }

    /* STEP-2 : Estimation */
    if ((obj -> MagnFluxEstEn == 1.0f) && (obj -> SlowDownEn == 0.0f))
       {  obj -> CtrlCycleCounter++;
          /* Limited badwidth differentiation. Backward Euler differentiation with followed by a 1st order Low-Pass Filter which is also discretized with Backward Euler. */
       //obj -> IqFbkDerivative += _IQmpy( obj -> LPF_IqDeriv_Gain, (_IQdiv((obj -> IqFbk - obj -> IqFbkPast), obj -> TimePU) - obj -> IqFbkDerivative) );
         obj -> IqFbkDerivative += ( obj -> LPF_IqDeriv_Gain * (((obj -> IqFbk - obj -> IqFbkPast) / obj -> TimePU) - obj -> IqFbkDerivative) );

          /* Pass variables to the DFT or averager which selects X(k=0) harmonic component. */
          /* _IQmpy(v.AveragerInputGain) is to get DFT amplitude at DC without overflow and it is equivalent to '2^(30-GLOBAL_Q)/N'. */

         // v.SpeedFbk_Average += _IQmpy(v.SpeedFbk, v.AveragerInputGain);
         // v.BackEMF_Average += _IQmpy( (v.VqRef - _IQmpy(v.Rs, v.IqFbk) - _IQmpy(v.Lq, v.IqFbkDerivative)), v.AveragerInputGain);

         obj -> SpeedFbk_Average += (obj -> SpeedFbk * obj -> AveragerInputGain);
         obj -> BackEMF_Average += ( (obj -> VqRef - (obj -> Rs * obj -> IqFbk) - (obj -> Lq * obj -> IqFbkDerivative)) * obj -> AveragerInputGain);

          /* STOP estimation when cycle number hits the predefined limit value of 'AveragingSize' */
          if (obj -> CtrlCycleCounter >= (float)(obj -> AveragingSize))
             {  obj -> CtrlCycleCounter = 0.0f;
                obj -> SpeedRampArrives = 0.0f;
                obj -> MagnFluxEstEn = 0.0f;
                obj -> SlowDownEn = 1.0f;

                obj -> SpeedRef = 0.0f;
                obj -> SpeedErrFiltered = (0.0f - obj -> TestSpeed); /* Precaution for pre-mature finish before truely slow-down due to the if-comparison in STEP-3. */

                /* Save the estimated permanent-magnet flux linkage */
                obj -> MagnFlux  = (obj -> BackEMF_Average / obj -> SpeedFbk_Average);
             }
       }

    /* Put present current feedback into memory for one sample delay. Used in the 'IqFbkDerivative'. */
    obj -> IqFbkPast = obj -> IqFbk;

    /* STEP-3 : Finish permanent-magnet flux linkage identification when motor speed stabilizes to zero. */
    if ((obj -> SpeedRampArrives == 1.0f) && (obj -> SlowDownEn == 1.0f))
       {  obj -> CtrlCycleCounter++;

          if (obj -> CtrlCycleCounter >= obj -> WaitingTime2Settle_Speed)
             {  /* Option-1 : Only time-based decision making. */
                /* v.Finish = TRUE; */
                /* v.VariableInit = TRUE; */

                /* Option-2 : Checking Speed error vanishes as a second parameter on top of the 'WaitingTime2Settle_Speed' timer for decision making. */
                /* This time, we do not set 'TimeOutError' even if settling takes more time than 'MaxTime4Settle' because estimation is already done and only stopping is left. */
                if ((MATH_abs(obj -> SpeedErrFiltered) < obj -> SpeedErrLim4Settle) || (obj -> CtrlCycleCounter >= obj -> MaxTime4Settle))
                   {
                      obj -> Finish = 1.0f;
                      obj -> VariableInit = 1.0f;
                   }
             }
       }


    return;
} // end of fluxlinest_run_series() function

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
