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
#include <math.h>
#include "math_types.h"

/*************************************************************************************************************************************************************************************/
// Defines the SATINDEST object
/*************************************************************************************************************************************************************************************/
typedef struct _SATINDEST_Obj_
{

    float32_t VariableInit;                 // Data: TRUE/FALSE. Estimation variables initialization enable.
    float32_t ExecuteEnable;                // Data: TRUE/FALSE. Parameter calculations are enabled.
    float32_t InjSineEnable;                // Data: TRUE/FALSE. Sinuoidal current injection is enabled.
    float32_t CurrentSign;                  // Data: Number. Positive=(+1), Negative=(-1).
    float32_t Axis;                         // Data: Number. D_AXIS=1, Q_AXIS=2.
    float32_t OpPtNo;                       // Data: Number. Magnetic operating point number.
    float32_t EstStep;                      // Data: Number. Step number in the estimation routine for that operating point.
    float32_t CtrlCycleCounter;             // Data: Number. Counter for control cycle entries here or execution of this MACRO.
    float32_t InjSampleCounter;             // Data: Number. Counter for injected sample numbers of sinusoidal injection current.
    float32_t InjSinePrdCounter;            // Data: Number. Counter for completed periods of injected sinusoidal current.
    float32_t InjCurHalfPrdSampleNo;        // Data: Number. Half of sample number of injected sinusoidal current in its one period.
    float32_t dcCurHalfPrdSampleNo;         // Data: Number. Half of sample number of square-wave current to determine magnetic operating point and keep rotor stationary.

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

    float32_t MotorType;                    // Parameter: Number. IM=1, IPMSM=2, SynRM=3.
    float32_t Nbase;                        // Parameter: Number. Base sample size: Fsampling/Fbase.
    float32_t InjSettleWait_PrdNo;          // Parameter: Number. Number of injected sinusoidal current periods to be completed for settling.
    float32_t dcSettleWait_SampleNo;        // Parameter: Number. Number of sample number to be completed for DC current settling after reference changes.
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


} SATINDEST_Obj;

/*****************************************************************************/
//  Defines the SATINDEST handle
/*****************************************************************************/

typedef struct _SATINDEST_Obj_ *SATINDEST_Handle;

/*****************************************************************************/

extern SATINDEST_Handle SATINDEST_init(void *pMemory, const size_t numBytes);

extern void SATINDEST_setParams(SATINDEST_Handle handle);

/*****************************************************************************/

//static inline void
//SELFCOMMM_run(SELFCOMMM_Handle handle, const float32_t vdc_ref,
//       const float32_t id_fb, const float32_t iq_fb, float32_t *pOutValue1, float32_t *pOutValue2, float32_t *pOutValue3)

// fonksiyonun inputlarini girmeyi unutma

static inline void SATINDEST_run(SATINDEST_Handle handle, const float32_t vdc_ref, const float32_t id_fb, const float32_t iq_fb)
{

    SATINDEST_Obj *obj = (SATINDEST_Obj *)handle;

    if (obj.VariableInit == 1.0f)                                                                                                                                                                  \
           {  obj.VariableInit = FALSE;                                                                                                                                                                   \
              v.ExecuteEnable = FALSE;                                                                                                                                                                  \
              v.InjSineEnable = FALSE;                                                                                                                                                                  \
              v.OpPtNo = 0;                                                                                                                                                                             \
              v.Axis = D_AXIS; /* Initialize. */                                                                                                                                                        \
              v.EstStep = LdqEstStep_POSITIVE_CURRENT_ESTIMATION; /* To shorthen the test. As Idc is zero at the beginning (OpPtNo is initialized to zero), no need for neg current.*/                  \
                                                                                                                                                                                                        \
              v.CtrlCycleCounter = 0;                                                                                                                                                                   \
              v.InjSampleCounter = 0;                                                                                                                                                                   \
              v.InjSinePrdCounter = 0;                                                                                                                                                                  \
                                                                                                                                                                                                        \
              /* Easy division for fixed-point implementation: InjCurHalfPrdSampleNo = 0.5*Fs/InjCurFreq. _IQ0 = _IQmpy(_IQ0, _IQN) */                                                                  \
              v.InjCurHalfPrdSampleNo = _IQdiv2(_IQmpy(v.Nbase, _IQdiv(_IQ(1.0), v.InjCurFreqInit)) ); /* Here, '_IQdiv' won't overflow because InjCurFreq > 1 p.u. mostly. */                          \
                                                                                                                                                                                                        \
              if ( (_IQdiv2(_IQmpy(v.Nbase, _IQdiv(_IQ(1.0), v.InjCurFreqInit)) ) - v.InjCurHalfPrdSampleNo) >= _IQ(0.5) ) /* Round the float variable into integer. Useful only in FLOAT_MATH */       \
                 {  v.InjCurHalfPrdSampleNo++;}                                                                                                                                                         \
                                                                                                                                                                                                        \
              /* Calculate the count number for half period of square wave current: DC settling count + (AC settling prd + DFT period)*AC Period count */                                               \
              v.dcCurHalfPrdSampleNo = v.dcSettleWait_SampleNo + (v.InjSettleWait_PrdNo + 1)*(2*v.InjCurHalfPrdSampleNo);                                                                               \
                                                                                                                                                                                                        \
              /* Notice that '_IQdiv' works for division of normal intergers as well as '_iq' format. */                                                                                                \
              v.InjCurAngleStep = _IQdiv( (1UL), (2*v.InjCurHalfPrdSampleNo) );                                                                                                                         \
                                                                                                                                                                                                        \
              /* We can update inj. freq. after selecting 'InjCurHalfPrdSampleNo' because exact freq. is now determined with 'InjCurHalfPrdSampleNo'. */                                                \
              v.InjCurFreq = _IQdiv(v.InjCurAngleStep, _IQdiv((1UL), v.Nbase) ); /* 1/Nbase = Fbase/Fs */                                                                                               \
                                                                                                                                                                                                        \
              v.InjCurAngle = _IQ(0.0);                                                                                                                                                                 \
              v.VrealDFT = _IQ(0.0);                                                                                                                                                                    \
              v.VimagDFT = _IQ(0.0);                                                                                                                                                                    \
              v.IrealDFT = _IQ(0.0);                                                                                                                                                                    \
              v.IimagDFT = _IQ(0.0);                                                                                                                                                                    \
                                                                                                                                                                                                        \
              /* 'InputGainDFT' is to prevent overflow in DFT memories. Normally we need to divide by (N/2). (1UL<<(30-GLOBAL_Q)) is to use all 32-bit capacity. */                                     \
              v.InputGainDFT = _IQmpy2(v.InjCurAngleStep);                                                                                                                                              \
                                                                                                                                                                                                        \
              v.Finish = FALSE;                                                                                                                                                                         \
           }

  /**********************************************************************************/

    if (obj->VariableInit == 1.0f)
       {
       obj->VariableInit = 0.0f;
       obj->VoltageFound = 0.0f;
       obj->VoltageNotFound = 0.0f;
       obj->VolMinAssigned = 0.0f;
       obj->VolMaxAssigned = 0.0f;
       obj->ExecuteEnable = 0.0f;

       obj->InjPrdCounter = 0.0f;
       obj->InjSampleCounter = 0.0f;
       obj->FirstLdqEstCheck = 0.0f;
       obj->FirstVolMaxLimHitCheck = 0.0f;
       obj->FirstFreqMinLimHitCheck = 0.0f;

       obj->InjVolHalfPrdSampleNo = obj->InjVolHalfPrdSampleNoInit;
          /* _IQdiv can be used for division of integers. */
//          InjVolAngleStep = _IQdiv( (1UL), (2*InjVolHalfPrdSampleNo) );
       obj->InjVolAngleStep = (1.0f/(2.0f*obj->InjVolHalfPrdSampleNo));

       obj->InjVolMagn = obj->InjVolMagnInit;
//          InjVolFreq = _IQdiv(InjVolAngleStep, AngIntgGain);
       obj->InjVolFreq = obj->InjVolAngleStep/obj->AngIntgGain;
       obj->InjVolAngle = 0.0f;
       obj->InjVolAngle_ZOHComp = 0.0f;
//          InputGainDFT = _IQmpy2(InjVolAngleStep);
       obj->InputGainDFT = 2.0f*obj->InjVolAngleStep;

       obj->VdRealDFT = 0.0f;
       obj->VdImagDFT = 0.0f;
       obj->IdRealDFT = 0.0f;
       obj->IdImagDFT = 0.0f;
       obj->IqRealDFT = 0.0f;
       obj->IqImagDFT = 0.0f;
       obj->ElecThetaUnsat = 0.0f;

       obj->ElecTheta = 0.0f;
       obj->Finish = 0.0f;

       obj->i12 = 0;
       }



    obj->Udc=vdc_ref/obj->Vdc_Base;
    obj->IdFbk=id_fb/obj->I_Base;
    obj->IqFbk=iq_fb/obj->I_Base;



//
//
//
    /* DISCRETE FOURIER TRANSFORM: Find the amplitude and phase of the voltage and current at the injection frequency. */
    /* Multiplication by 'InputGainDFT' is to get DFT amplitude and equivalent to dividing by N/2. 'InputGainDFT' is updated when frequency changes. So, no problem. */
    if (obj->InjPrdCounter == obj->InjSettlePrd)
       {
//          CosDFT = _IQcosPU(InjVolAngle);
//          SinDFT = _IQsinPU(InjVolAngle);

//        obj->CosDFT = cosf(obj->InjVolAngle*MATH_TWO_PI);
//        obj->SinDFT = sinf(obj->InjVolAngle*MATH_TWO_PI);

        obj->CosDFT = cosf(obj->InjVolAngle*MATH_TWO_PI);
        obj->SinDFT = sinf(obj->InjVolAngle*MATH_TWO_PI);

//          VdRealDFT += _IQmpy(_IQmpy(_IQmpy(_IQmpy(InjVolMagn, _IQcosPU(InjVolAngle)), Udc), CosDFT), InputGainDFT);
//          VdImagDFT -= _IQmpy(_IQmpy(_IQmpy(_IQmpy(InjVolMagn, _IQcosPU(InjVolAngle)), Udc), SinDFT), InputGainDFT);

        obj->VdRealDFT += obj->InjVolMagn* cosf(obj->InjVolAngle*MATH_TWO_PI)* obj->Udc* obj->CosDFT* obj->InputGainDFT;
        obj->VdImagDFT -= obj->InjVolMagn* cosf(obj->InjVolAngle*MATH_TWO_PI)* obj->Udc* obj->SinDFT* obj->InputGainDFT;

//          IdRealDFT += _IQmpy(_IQmpy(IdFbk, CosDFT), InputGainDFT);
//          IdImagDFT -= _IQmpy(_IQmpy(IdFbk, SinDFT), InputGainDFT);
//
//          IqRealDFT += _IQmpy(_IQmpy(IqFbk, CosDFT), InputGainDFT);
//          IqImagDFT -= _IQmpy(_IQmpy(IqFbk, SinDFT), InputGainDFT);

        obj->IdRealDFT += obj->IdFbk* obj->CosDFT*obj->InputGainDFT;
        obj->IdImagDFT -= obj->IdFbk* obj->SinDFT*obj->InputGainDFT;

        obj->IqRealDFT += obj->IqFbk* obj->CosDFT*obj->InputGainDFT;
        obj->IqImagDFT -= obj->IqFbk* obj->SinDFT*obj->InputGainDFT;
       }
//
    /* INJECTION PERIOD HANDLER: Enables signal detection by DFT and inductance calculation. */
    /* In this ctrl cycle, the last sample of the injected voltage period  is being applied. */
    /* At the end of this ctrl cycle, there will be zero-crossing of the current due to almost 90 degree phase shift at high frequency. Thus, if V or f changes now, no transient is seen. */
    if ( obj->InjSampleCounter == (2.0f*obj->InjVolHalfPrdSampleNo) )
       {  obj->InjSampleCounter = 0.0f;
       obj->InjPrdCounter++;
          /* Just after the injection period for DFT finishes, enable execution. */
          if (obj->InjPrdCounter == (obj->InjSettlePrd + 1.0f))
             {  obj->InjPrdCounter = 0.0f;
                obj->ExecuteEnable = 1.0f;}
       }
//
//
    /* MAKE CALCULATIONS */
    if (obj->ExecuteEnable == 1.0f)
       {  obj->ExecuteEnable = 0.0f;

//          /* 1- SIGNAL AMPLITUDE and PHASE CALCULATION */
//          /* D-axis injection voltage amplitude and phase */
//          VdMagnDFT = _IQsqrt( _IQmpy(VdRealDFT, VdRealDFT) + _IQmpy(VdImagDFT, VdImagDFT) );
//          VdPhaseDFT = _IQatan2PU(VdImagDFT, VdRealDFT);
//
//          /* D-axis current amplitude and phase. */
//          IdMagnDFT = _IQsqrt( _IQmpy(IdRealDFT, IdRealDFT) + _IQmpy(IdImagDFT, IdImagDFT) );
//          IdPhaseDFT = _IQatan2PU(IdImagDFT, IdRealDFT);
//
//          /* Q-axis current amplitude. */
//          IqMagnDFT = _IQsqrt( _IQmpy(IqRealDFT, IqRealDFT) + _IQmpy(IqImagDFT, IqImagDFT) );

            /* 1- SIGNAL AMPLITUDE and PHASE CALCULATION */
            /* D-axis injection voltage amplitude and phase */

//       obj->VdMagnDFT = ti_arm_sqrt((obj->VdRealDFT* obj->VdRealDFT) + (obj->VdImagDFT* obj->VdImagDFT));
//       obj->VdPhaseDFT = ti_arm_atan2(obj->VdImagDFT,obj->VdRealDFT);
//
//            /* D-axis current amplitude and phase. */
//       obj->IdMagnDFT = ti_arm_sqrt( (obj->IdRealDFT* obj->IdRealDFT) + (obj->IdImagDFT* obj->IdImagDFT) );
//       obj->IdPhaseDFT = ti_arm_atan2(obj->IdImagDFT, obj->IdRealDFT);
//
//            /* Q-axis current amplitude. */
//       obj->IqMagnDFT = ti_arm_sqrt( (obj->IqRealDFT* obj->IqRealDFT) + (obj->IqImagDFT* obj->IqImagDFT) );

       obj->VdMagnDFT = sqrtf((obj->VdRealDFT* obj->VdRealDFT) + (obj->VdImagDFT* obj->VdImagDFT));
       obj->VdPhaseDFT = atan2f(obj->VdImagDFT,obj->VdRealDFT);

            /* D-axis current amplitude and phase. */
       obj->IdMagnDFT = sqrtf( (obj->IdRealDFT* obj->IdRealDFT) + (obj->IdImagDFT* obj->IdImagDFT) );
       obj->IdPhaseDFT = atan2f(obj->IdImagDFT, obj->IdRealDFT);

            /* Q-axis current amplitude. */
       obj->IqMagnDFT = sqrtf( (obj->IqRealDFT* obj->IqRealDFT) + (obj->IqImagDFT* obj->IqImagDFT) );




          /* Reset DFT variables. */
       obj->VdRealDFT = 0.0f;
       obj->VdImagDFT = 0.0f;
       obj->IdRealDFT = 0.0f;
       obj->IdImagDFT = 0.0f;
       obj->IqRealDFT = 0.0f;
       obj->IqImagDFT = 0.0f;

          /* Stator current vector amplitude. */
       obj->IsMagn = ((obj->IdMagnDFT* obj->IdMagnDFT) + (obj->IqMagnDFT* obj->IqMagnDFT));
//       obj->IsMagn = 0.1f;

          /* If 'IsMagn' leaves the allowed range at any electrical angle, find a new injection signal. */
          if ((obj->VoltageFound == 1.0f) && ((obj->IsMagn < obj->IsMagnMinLim) || (obj->IsMagn > obj->IsMagnMaxLim)))
             {  obj->VoltageFound = 0.0f;
             obj->VolMinAssigned = 0.0f;
             obj->VolMaxAssigned = 0.0f;
             obj->MinSearchVol = 0.0f;
             obj->MaxSearchVol = 0.0f;}


          /* 2- INJECTION SIGNAL SELECTION */
          /* Determine the injection voltage magnitude and frequency.  */
          if (obj->VoltageFound == 0.0f)
             {
                if (obj->IsMagn < obj->IsMagnMinLim)
                   {  obj->MinSearchVol = obj->InjVolMagn;
                   obj->VolMinAssigned = 1.0f;

                      if (obj->VolMaxAssigned == 0.0f)
                         {  //InjVolMagn = _IQmpy2(InjVolMagn);
                          obj->InjVolMagn = 2.0f*(obj->InjVolMagn);

                            /* Check limits. */
                            if (obj->InjVolMagn > obj->InjVolMagnMaxLim)
                               {  obj->InjVolMagn = obj->InjVolMagnMaxLim;
                               obj->FirstVolMaxLimHitCheck++;

                                  /* If max voltage is not enough, then decrese the frequency. */
                                  if (obj->FirstVolMaxLimHitCheck > 1.0f)
                                     {  /* To prevent uncontrolled rise and overflow, limit 'FirstVolMaxLimHitCheck' to 2 which is higher than 1. So, it still satisfies above IF. */
                                      obj->FirstVolMaxLimHitCheck = 2.0f;
                                        /* Reset VolMin/Max and start-over again if freq change occurs. */
                                      obj->VolMinAssigned = 0.0f;
                                      obj->VolMaxAssigned = 0.0f;
                                        /* InjVolMagn = InjVolMagnInit; */ /* Check this line, either use or not. */

                                      obj->InjVolHalfPrdSampleNo = 2.0f*obj->InjVolHalfPrdSampleNo;
//                                        InjVolAngleStep = _IQdiv2(InjVolAngleStep/2.0f);
//                                        InjVolFreq = _IQdiv2(InjVolFreq);
//                                        InputGainDFT = _IQdiv2(InputGainDFT);

                                      obj->InjVolAngleStep = (obj->InjVolAngleStep/2.0f);
                                      obj->InjVolFreq = (obj->InjVolFreq/2.0f);
                                      obj->InputGainDFT = (obj->InputGainDFT/2.0f);

                                        if (obj->InjVolHalfPrdSampleNo > obj->InjVolHalfPrdSampleNoMax)
                                           {  obj->InjVolHalfPrdSampleNo = obj->InjVolHalfPrdSampleNoMax;
                                              /* _IQdiv can be used for division of integers. */
                                           obj-> InjVolAngleStep = ((1.0f)/ (2.0f*obj->InjVolHalfPrdSampleNo));
                                           obj->InjVolFreq = (obj->InjVolAngleStep/ obj->AngIntgGain);
                                           obj->InputGainDFT = (2.0f*obj->InjVolAngleStep);

                                           obj->FirstFreqMinLimHitCheck++;
                                              if (obj->FirstFreqMinLimHitCheck > 1.0f)
                                                 {  /* To prevent uncontrolled rise and overflow, limit 'FirstFreqMinLimHitCheck' to 2 which is higher than 1. So, it still satisfies above IF. */
                                                  obj->FirstFreqMinLimHitCheck = 2.0f;
                                                  obj->VoltageFound = 1.0f;
                                                  obj->VoltageNotFound = 1.0f;} /* DONT FORGET TO USE THIS ERROR MESSAGE. */
                                           }
                                      }
                                }
                          }
                       else
                          {  //InjVolMagn = _IQdiv2(MinSearchVol + MaxSearchVol);
                           obj->InjVolMagn = (obj->MinSearchVol + obj->MaxSearchVol)/2.0f;}
                   }

                else if (obj->IsMagn > obj->IsMagnMaxLim)
                   {  obj->MaxSearchVol = obj->InjVolMagn;
                   obj->VolMaxAssigned = 1.0f;

                      if (obj->VolMinAssigned == 0.0f)
                         {  //InjVolMagn = _IQdiv2(InjVolMagn);
                          obj->InjVolMagn = (obj->InjVolMagn)/2.0f;}
                      else
                         {  //InjVolMagn = _IQdiv2(MinSearchVol + MaxSearchVol);
                          obj->InjVolMagn = (obj->MinSearchVol + obj->MaxSearchVol)/2.0f;}
                   }
                else
                   {  obj->VoltageFound = 1.0f;}

             }


          /* 3- LD, LQ, THETA CALCULATION */
          /* Search for Ld, Lq, and initial rotor position. Scan 180 electrical degrees in rotor with small angle steps. */
          /* If Part-2 decides VoltageFound, then code directly enter here. No need to repeat the test. */
          if (obj->VoltageFound == 1.0f)
             {
                /* Calculate inductance. */
                //Lpu = _IQmpy(_IQdiv((VdMagnDFT), _IQmpy(InjVolFreq, IdMagnDFT)), _IQsinPU(VdPhaseDFT - IdPhaseDFT) );
              obj->Lpu = (((obj->VdMagnDFT)/(obj->InjVolFreq* obj->IdMagnDFT))* sinf(obj->VdPhaseDFT - obj->IdPhaseDFT));
              obj->Ind_Total[obj->i12]=obj->Lpu;
              obj->Ang_Total[obj->i12]=obj->ElecTheta;
              obj->i12++;
              obj->FirstLdqEstCheck++;
                if (obj->FirstLdqEstCheck == 1.0f)
                   {  obj->LminApprox = obj->Lpu;
                   obj->LmaxApprox = obj->Lpu;
                   obj->InitialRotorAngle = obj->ElecTheta;}

                /* To prevent uncontrolled rise and overflow, limit 'FirstLdqEstCheck' to 2 which is higher than 1. So, it still satisfies above IF-statement. */
                if (obj->FirstLdqEstCheck > 1.0f)
                   {  obj->FirstLdqEstCheck = 2.0f;}

                if (obj->Lpu > obj->LmaxApprox)
                   {  obj->LmaxApprox = obj->Lpu;}
                if (obj->Lpu < obj->LminApprox)
                   {  obj->LminApprox = obj->Lpu;
                   obj->InitialRotorAngle = obj->ElecTheta;}

                obj-> ElecThetaUnsat += obj->ElecThetaStep;

//                ElecTheta = _IQsat(ElecThetaUnsat, _IQ(0.5), 0.0f);
                obj->ElecTheta = MATH_sat(obj->ElecThetaUnsat, 0.5f, 0.0f);
             }
       }



    /* Compute the angle for injection sinusoidal voltage. */
    /* In order to synthesize real cosine voltage appering at the motor, the injection angle starts from ' - _IQdiv2(InjVolAngleStep)'. */
    obj->InjVolAngle = (obj->InjSampleCounter * obj->InjVolAngleStep);
    /* Saturate the angle within (0, 1) because of '_IQcosPU'. */
    while (obj->InjVolAngle > 1.0f)
        {  obj->InjVolAngle -= 1.0f;}
    while (obj->InjVolAngle < -1.0f)
        {  obj->InjVolAngle += 1.0f;}
    obj->InjSampleCounter++;

    /* The addition of _IQdiv2(InjVolAngleStep) is to compensate the phase delay due to the ZOH effect of PWM. So, motor experiences 1.0f cosine voltage. */
    /* Notice that both CMPA update from PWM shadow registers and SOCA/interrupt execution are initiated at TBCTR = PRD. This is important for phase difference between voltage and current. */
//    InjVolAngle_ZOHComp = InjVolAngle + _IQdiv2(InjVolAngleStep);
    obj->InjVolAngle_ZOHComp = obj->InjVolAngle + (obj->InjVolAngleStep/2.0f);
    while (obj->InjVolAngle_ZOHComp > 1.0f)
        {  obj->InjVolAngle_ZOHComp -= 1.0f;}
    while (obj->InjVolAngle_ZOHComp < -1.0f)
        {  obj->InjVolAngle_ZOHComp += 1.0f;}

    /* Calculate injection voltage. Put this part after Goertzel IIR part because this voltage will be applied during next ctrl cycle. */
    obj->InjVol = (obj->InjVolMagn* cosf(obj->InjVolAngle_ZOHComp*MATH_TWO_PI))*vdc_ref/sqrtf(3.0f);


    /* Stop. */
    if (obj->ElecThetaUnsat > obj->ElecTheta)
       {  obj->Finish = 1.0f;
          obj->VariableInit = 1.0f;}

//    *pOutValue1 =obj->InjVol;
//    *pOutValue2 =obj->ElecTheta;
//    *pOutValue3 =obj->Finish;

//
//    // Saturate the output
//#ifdef __TMS320C28XX_CLA__
//    *pOutValue = MATH_sat(Up + Ui, outMax, outMin);
//#else
//    *pOutValue = MATH_sat(Up + Ui, outMax, outMin);
//#endif  // __TMS320C28XX_CLA__

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
