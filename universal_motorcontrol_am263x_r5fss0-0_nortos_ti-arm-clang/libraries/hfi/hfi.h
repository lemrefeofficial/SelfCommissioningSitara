//#############################################################################
//
// FILE:   hfi.h
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

#ifndef HFI_H
#define HFI_H

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

#define   FLOAT_MATH     1
#define   IQ_MATH        0

#define   MATH_TYPE      FLOAT_MATH


/*************************************************************************************************************************************************************************************/
// Defines the HFI object
/*************************************************************************************************************************************************************************************/
typedef struct _HFI_Obj_
{
    float32_t VariableInit;                     // Data: TRUE/FALSE. Variable initialization is enabled.
    float32_t SignalSelectionError;             // Data: TRUE/FALSE. Injection signal cannot be selected under the limitation of min injection frequency and target current amplitude.
    float32_t GoertzelEnable;                   // Data: TRUE/FALSE. High-frequency current amplitude measurement algorithm for initial postion detection is enabled.
    float32_t MagnPolarDetectFinished;          // Data: TRUE/FALSE. Magnet polarity is detected. Initial postion detection is achieved.
    float32_t MagnPolarDetectMode;              // Data: Number. Meaning 0: Idle mode, 1: Negative Id, 2: Positive Id
    float32_t CtrlCycleCounter;                 // Data: Number. Counter for ctrl entry in this module, used for timing in initial rotor postion detection.
    float32_t InjSampleCounter;                 // Data: Number. Counter for injected sample numbers of sinusoidal injection voltage.
    float32_t GoertzelCounter;                  // Data: Number. Counter for samples used in High-frequency current amplitude measurement algorithm.
    float32_t Wait4SettleHFI;                   // Data: Number. Samples to wait for HFI algorithm to settle.
    float32_t Wait4SettleCurrent;               // Data: Number. Samples to wait for current loop to settle.
    float32_t InjVolHalfPrdSampleNo;            // Data: Number. Half of sample number of injected voltage in its one period - currently utilized value.
    float32_t InjVolAngleStep;                  // Data: Per-Unit. Angle steps to increment 'InjVolAngle'.
    float32_t InjVolMagn_UdcComp;               // Data: Per-Unit. Injected high-frequency voltage magnitude, DC-link voltage compensated.
    float32_t InjVolMagn;                       // Data: Per-Unit. Injected high-frequency voltage magnitude.
    float32_t InjVolFreq;                       // Data: Per-Unit. Injected high-frequency voltage frequency.
    float32_t Kerror;                           // Data: Per-Unit. Machine dependent postion estimator loop gain.
    float32_t HFI_Wc;                           // Data: Per-Unit. Designated postion estimator loop cross-over frequnecy.
    float32_t LPF_Error_Wc;                     // Data: Per-Unit. Cut-off frequency of 'LPF_Error'.
    float32_t LPF_SpeedEst_Wc;                  // Data: Per-Unit. Cut-off frequency of 'LPF_SpeedEst'.
    float32_t Kp_hfi;                           // Data: Gain. PI controller proportional gain.
    float32_t Ki_hfi;                           // Data: Gain. PI controller integral gain.
    float32_t Err;                              // Data: Per-Unit. PI controller error term.
    float32_t Intg;                             // Data: Per-Unit. PI controller integral term.
    float32_t aw;                               // Data: Per-Unit. PI controller anti-windup term.
    float32_t PIout;                            // Data: Per-Unit. PI controller output, unfiltered and unsaturated, raw speed estimation.
    float32_t SpeedEst;                         // Data: Per-Unit. PI controller output, unfiltered but saturated speed estimation.
    float32_t InjVolAngle;                      // Data: Per-Unit. Injected high-frequency voltage angle.
    float32_t DemodAngle;                       // Data: Per-Unit. Demodulation sinusoidal signal's angle for current at 1st harmonic of injection.
    float32_t BPF_Id_Out;                       // Data: Per-Unit. Output of 2nd order BPF to select Id at 1st harmonic of injection.
    float32_t BPF_Id_OutPast1;                  // Data: Per-Unit. One sample delayed 'BPF_Id_Out'.
    float32_t BPF_Id_OutPast2;                  // Data: Per-Unit. Two sample delayed 'BPF_Id_Out'.
    float32_t BPF_Id_InPast1;                   // Data: Per-Unit. One sample delayed 'IdFbk'.
    float32_t BPF_Id_InPast2;                   // Data: Per-Unit. Two sample delayed 'IdFbk'.
    float32_t BPF_Iq_Out;                       // Data: Per-Unit. Output of 2nd order BPF to select Iq at 1st harmonic of injection.
    float32_t BPF_Iq_OutPast1;                  // Data: Per-Unit. One sample delayed 'BPF_Iq_Out'.
    float32_t BPF_Iq_OutPast2;                  // Data: Per-Unit. Two sample delayed 'BPF_Iq_Out'.
    float32_t BPF_Iq_InPast1;                   // Data: Per-Unit. One sample delayed 'IqFbk'.
    float32_t BPF_Iq_InPast2;                   // Data: Per-Unit. Two sample delayed 'IqFbk'.
    float32_t LPF_Error_Out;                    // Data: Per-Unit. Output of 1st order LPF to select demoulated Iq amplitude at 1st harmonic of injection.
    float32_t BPF_GainIn;                       // Data: Gain. Multiplier of 'IqFbk' in 'BPF_Out' calculation.
    float32_t BPF_GainOut1;                     // Data: Gain. Multiplier of 'BPF_OutPast1' in 'BPF_Out' calculation.
    float32_t BPF_GainOut2;                     // Data: Gain. Multiplier of 'BPF_OutPast2' in 'BPF_Out' calculation.
    float32_t LPF_Error_Gain;                   // Data: Gain. Multiplier of correction term in 'LPF_Error_Out' calculation.
    float32_t LPF_SpeedEst_Gain;                // Data: Gain. Multiplier of correction term in 'SpeedEstFiltered' calculation.
    float32_t gGain;                            // Data: Gain. Goertzel Algorithm. The gain of 'gId_Past1'.
    float32_t gId_Now;                          // Data: Per-Unit. Goertzel Algorithm. Intermediate value in implementation via Direct Form 2.
    float32_t gId_Past1;                        // Data: Per-Unit. Goertzel Algorithm. One delayed gId_Now.
    float32_t gId_Past2;                        // Data: Per-Unit. Goertzel Algorithm. Two delayed gId_Now.
    float32_t InjIdMagn;                        // Data: Per-Unit. Amplitude of Id feedback due to high-frequency voltage injection.
    float32_t IdMagn_MPD_Mode1;                 // Data: Per-Unit. Amplitude of Id feedback due to high-frequency voltage injection - at Mode 1.
    float32_t IdMagn_MPD_Mode2;                 // Data: Per-Unit. Amplitude of Id feedback due to high-frequency voltage injection - at Mode 2.

    float32_t InjVolHalfPrdSampleNoInit;        // Parameter: Number. Half of sample number of injected voltage in its one period - initial value.
    float32_t InjVolHalfPrdSampleNoMaxLim;      // Parameter: Number. Allowed max value for injected voltage half period sample number i.e. minimum injection frequency.
    float32_t SampleNoBase;                     // Parameter: Number. Sample number in one base period: Fsample/Fbase.
    float32_t InjVolMagnMaxLim;                 // Parameter: Per-Unit. Allowed max value for injected voltage magnitude.
    float32_t IhfiTarget;                       // Parameter: Per-Unit. Target current due to high-frequency voltage injection
    float32_t UdcCompMinVol;                    // Parameter: Per-Unit. Min Dc-link voltage for DC-link conpensation.
    float32_t Ratio_InjFreq_WcHFI;              // Parameter: Per-Unit. Ratio of Injection frequency over HFI cross-over frequency: Finj/FcHFI.
    float32_t Ratio_WcHFI_WcLPF;                // Parameter: Per-Unit. Ratio of HFI cross-over frequency over error low-pass filter cut-off frequency: FcHFI/FcLPF.
    float32_t Ratio_WcCurrent_WcHFI;            // Parameter: Per-Unit. Designated current loop cross-over frequnecy.
    float32_t SpeedEstMaxLim;                   // Parameter: Per-Unit. Speed estimation allowed maximum value.
    float32_t BPF_Zeta;                         // Parameter: Gain. Damping factor zeta of 'BPF'.
    float32_t AngIntgGain;                      // Parameter: Gain. Angle integration gain: Fbase*Tsample.
    float32_t TimePU;                           // Parameter: Per-Unit. Per-Unit value of sampling time.
    float32_t Ld;                               // Parameter: Per-Unit. D-axis inductance.
    float32_t Lq;                               // Parameter: Per-Unit. Q-axis inductance.

    float32_t IdFbk;                            // Input: Per-Unit. D-axis current feedback.
    float32_t IqFbk;                            // Input: Per-Unit. Q-axis current feedback.
    float32_t Udc;                              // Input: Per-unit. DC-link voltage.
    float32_t ff_SpeedRef;                      // Input: Per-Unit. PI controller feed-forward term, refrence of the speed controller.

    float32_t InjVol;                           // Output: Per-Unit. Injected high-frequency voltage.
    float32_t SpeedEstFiltered;                 // Output: Per-Unit. Estimated electrical speed (low-pass filtered).
    float32_t ElecThetaEst;                     // Output: Per-Unit. Estimated electrical angle.

    float32_t Vdc_Base;
    float32_t Vph_Base;
    float32_t I_Base;
    float32_t F_Base;

    float32_t BASE_FREQ;

} HFI_Obj;

/*****************************************************************************/
//  Defines the HFI handle
/*****************************************************************************/

typedef struct _HFI_Obj_ *HFI_Handle;

/*****************************************************************************/

extern HFI_Handle HFI_init(void *pMemory, const size_t numBytes);

extern void HFI_setParams(HFI_Handle handle);

/*****************************************************************************/

// dont forget to define function inputs.

static inline void HFI_run(HFI_Handle handle, const float32_t Id_fb, const float32_t Iq_fb, const float32_t Vdc, const float32_t feedforwardSpeedref)
{

    HFI_Obj *obj = (HFI_Obj *)handle;

        /* Initialize some variables. */
        /* This enables safe start and usage of this routine again without resetting CPU. */

    if (obj -> VariableInit == 1.0f)
       {  obj -> VariableInit = 0.0f;

          obj -> SignalSelectionError = 0.0f;
          obj -> GoertzelEnable = 0.0f;
          obj -> MagnPolarDetectFinished = 0.0f;
          obj -> MagnPolarDetectMode = 0.0f;
          obj -> CtrlCycleCounter = 0.0f;
          obj -> InjSampleCounter = 0.0f;
          obj -> GoertzelCounter = 0.0f;

          obj -> InjVolHalfPrdSampleNo = obj -> InjVolHalfPrdSampleNoInit - 1.0f;
          do {  obj -> InjVolHalfPrdSampleNo++;
                if (obj -> InjVolHalfPrdSampleNo > obj -> InjVolHalfPrdSampleNoMaxLim)
                   {  obj -> InjVolHalfPrdSampleNo = obj -> InjVolHalfPrdSampleNoMaxLim;
                      obj -> InjVolMagnMaxLim = obj -> InjVolMagn;    /* Precaution to break the possible endless loop. */
                      obj -> SignalSelectionError = 1.0f; /* DONT FORGET TO USE THIS ERROR MESSAGE. */
                   }

                /* '_IQdiv' can work for the division of two integer variables. */
                obj -> InjVolAngleStep = (1.0f / (2.0f * obj -> InjVolHalfPrdSampleNo));
                obj -> InjVolFreq = (obj -> InjVolAngleStep / obj -> AngIntgGain);

                obj -> InjVolMagn = ((obj -> InjVolFreq * obj -> Ld) * obj -> IhfiTarget);
             }
          while (obj -> InjVolMagn > obj -> InjVolMagnMaxLim);
          // obj -> Kerror = _IQdiv( _IQmpy(obj -> InjVolMagn, _IQdiv2(obj -> Lq - obj -> Ld)), _IQmpy(obj -> InjVolFreq, _IQmpy(obj -> Lq, obj -> Ld)) );

          obj -> Kerror = ( (obj -> InjVolMagn * ((obj -> Lq - obj -> Ld) / 2.0f)) / (obj -> InjVolFreq * (obj -> Lq * obj -> Ld)) );
          obj -> HFI_Wc = (obj -> InjVolFreq / obj -> Ratio_InjFreq_WcHFI);

          /* OVERFLOW TEST FOR FIXED-POINT IMPLEMENTATION: Decrease target 'obj -> HFI_Wc'. */
          if (MATH_TYPE == IQ_MATH)
             {  obj -> Kp_hfi = (obj -> Kerror / obj -> HFI_Wc); /* In reality, it is 1/Kp which is small and hence safe. */
                while ( obj -> Kp_hfi < ( 1.0f / ((float)(1UL << (30 - 24))) ) )
                    {  obj -> Kp_hfi = (2.0f * obj -> Kp_hfi);
                       obj -> HFI_Wc = (obj -> HFI_Wc / 2.0f);
                       obj -> Ratio_InjFreq_WcHFI = (obj -> Ratio_InjFreq_WcHFI * 2.0f);} /* Update the ratio which is now utilized. */
             }

          obj -> Kp_hfi = (obj -> HFI_Wc / obj -> Kerror); /* CHECKED FOR OVERFLOW. */
          obj -> Ki_hfi = (obj -> TimePU * (obj -> HFI_Wc / obj -> Ratio_WcHFI_WcLPF));

          obj -> LPF_Error_Wc = (obj -> HFI_Wc * obj -> Ratio_WcHFI_WcLPF);
          obj -> LPF_SpeedEst_Wc = obj -> HFI_Wc;

          /* Current loop need 1.5 multiples and position estimator needs 2 multiples. But it is secure to use 3 as multiplier. */
          obj -> Wait4SettleCurrent = 3.0f * (obj -> SampleNoBase * ((1.0f) / (obj -> Ratio_WcCurrent_WcHFI * obj -> HFI_Wc)));
          obj -> Wait4SettleHFI = 3.0f * (obj -> SampleNoBase * ((1.0f) / obj -> HFI_Wc));

          obj -> Intg = 0.0f;
          obj -> aw = 0.0f;

          obj -> InjVolAngle = -(obj -> InjVolAngleStep / 2.0f); /* Injection angle starts half sample positive to consider ZOH nature of the inverter. */
          obj -> DemodAngle = -obj -> InjVolAngleStep; /* Start one sample shifted to consider the computation delay. */
          obj -> BPF_Id_OutPast1 = 0.0f;
          obj -> BPF_Id_OutPast2 = 0.0f;
          obj -> BPF_Id_InPast1 = 0.0f;
          obj -> BPF_Id_InPast2 = 0.0f;
          obj -> BPF_Iq_OutPast1 = 0.0f;
          obj -> BPF_Iq_OutPast2 = 0.0f;
          obj -> BPF_Iq_InPast1 = 0.0f;
          obj -> BPF_Iq_InPast2 = 0.0f;
          /* Filter design. */
          obj -> BPF_GainOut1 = ( (2.0f * cosf(obj -> InjVolAngleStep * MATH_TWO_PI )) / ((1.0f) + (obj -> BPF_Zeta * sinf(obj -> InjVolAngleStep * MATH_TWO_PI))) );

        //obj -> BPF_GainOut2 = _IQdiv( ( _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) - _IQ(1.0) ), ( _IQ(1.0) + _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) ) );
          obj -> BPF_GainOut2 = ( ( (obj -> BPF_Zeta * sinf(obj -> InjVolAngleStep * MATH_TWO_PI )) - (1.0f) ) / ( (1.0f) + (obj -> BPF_Zeta * sinf(obj -> InjVolAngleStep * MATH_TWO_PI)) ) );

        //obj -> BPF_GainIn = _IQdiv( ( _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) ), ( _IQ(1.0) + _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) ) );
          obj -> BPF_GainIn = ( ( (obj -> BPF_Zeta * sinf(obj -> InjVolAngleStep * MATH_TWO_PI)) ) / ( (1.0f) + (obj -> BPF_Zeta * sinf(obj -> InjVolAngleStep * MATH_TWO_PI)) ) );

        //obj -> LPF_Error_Gain = _IQdiv(_IQmpy(obj -> LPF_Error_Wc, obj -> TimePU), (_IQ(1.0) + _IQmpy(obj -> LPF_Error_Wc, obj -> TimePU)));
          obj -> LPF_Error_Gain = ((obj -> LPF_Error_Wc * obj -> TimePU) / ((1.0f) + (obj -> LPF_Error_Wc * obj -> TimePU)));

        //obj -> LPF_SpeedEst_Gain = _IQdiv(_IQmpy(obj -> LPF_SpeedEst_Wc, obj -> TimePU), (_IQ(1.0) + _IQmpy(obj -> LPF_SpeedEst_Wc, obj -> TimePU)));
          obj -> LPF_SpeedEst_Gain = ((obj -> LPF_SpeedEst_Wc * obj -> TimePU) / ((1.0f) + (obj -> LPF_SpeedEst_Wc * obj -> TimePU)));

        /* Goertzel variables. */
        //obj -> gGain = _IQmpy2(_IQcosPU(obj -> InjVolAngleStep));
          obj -> gGain = (2.0f * cosf(obj -> InjVolAngleStep * MATH_TWO_PI));
          obj -> gId_Past1 = (0.0f);
          obj -> gId_Past2 = (0.0f);

          obj -> SpeedEstFiltered = (0.0f);
       }

    obj -> IdFbk =  Id_fb / obj->I_Base;
    obj -> IqFbk =  Iq_fb / obj->I_Base;
    obj -> Udc =  Vdc /obj->Vdc_Base;
    obj -> ff_SpeedRef = feedforwardSpeedref  / obj -> F_Base;

    /* Extract the harmonic current in the Estimated Q-axis */
    /* 2nd order Band-Pass Filter - Direct Form 1 */
    /* Tustin discretization - Prewarped at the BPF center freqency for zero phase lag there */
  //obj -> BPF_Iq_Out = _IQmpy(obj -> BPF_GainIn, (obj -> IqFbk - obj -> BPF_Iq_InPast2)) + _IQmpy(obj -> BPF_GainOut1, obj -> BPF_Iq_OutPast1) + _IQmpy(obj -> BPF_GainOut2, obj -> BPF_Iq_OutPast2);
    obj -> BPF_Iq_Out = (obj -> BPF_GainIn * (obj -> IqFbk - obj -> BPF_Iq_InPast2)) + (obj -> BPF_GainOut1 * obj -> BPF_Iq_OutPast1) + (obj -> BPF_GainOut2 * obj -> BPF_Iq_OutPast2);

    obj -> BPF_Iq_OutPast2 = obj -> BPF_Iq_OutPast1;
    obj -> BPF_Iq_OutPast1 = obj -> BPF_Iq_Out;
    obj -> BPF_Iq_InPast2 = obj -> BPF_Iq_InPast1;
    obj -> BPF_Iq_InPast1 = obj -> IqFbk;

  //obj -> BPF_Id_Out = _IQmpy(obj -> BPF_GainIn, (obj -> IdFbk - obj -> BPF_Id_InPast2)) + _IQmpy(obj -> BPF_GainOut1, obj -> BPF_Id_OutPast1) + _IQmpy(obj -> BPF_GainOut2, obj -> BPF_Id_OutPast2);
    obj -> BPF_Id_Out = (obj -> BPF_GainIn * (obj -> IdFbk - obj -> BPF_Id_InPast2)) + (obj -> BPF_GainOut1 * obj -> BPF_Id_OutPast1) + (obj -> BPF_GainOut2 * obj -> BPF_Id_OutPast2);

    obj -> BPF_Id_OutPast2 = obj -> BPF_Id_OutPast1;
    obj -> BPF_Id_OutPast1 = obj -> BPF_Id_Out;
    obj -> BPF_Id_InPast2 = obj -> BPF_Id_InPast1;
    obj -> BPF_Id_InPast1 = obj -> IdFbk;

  /* By Low-pass Filtering, Extract the DC Error due to Estimated and Actual Axis Mismatch  */
  /* 1st order Low-Pass Filter - Backward Euler discretization */
  //obj -> LPF_Error_Out += _IQmpy(obj -> LPF_Error_Gain, (_IQmpy(obj -> BPF_Iq_Out, _IQsinPU(obj -> DemodAngle)) - obj -> LPF_Error_Out));
    obj -> LPF_Error_Out += (obj -> LPF_Error_Gain * ((obj -> BPF_Iq_Out * sinf(obj -> DemodAngle * MATH_TWO_PI)) - obj -> LPF_Error_Out));

    /* PI Controller */
  //obj -> Err = _IQmpy(obj -> Kp_hfi, obj -> LPF_Error_Out);
    obj -> Err = (obj -> Kp_hfi * obj -> LPF_Error_Out);

  //obj -> Intg += _IQmpy(obj -> Ki_hfi, (obj -> Err - obj -> aw));
    obj -> Intg += (obj -> Ki_hfi * (obj -> Err - obj -> aw));

  //obj -> PIout = obj -> ff_SpeedRef +  (obj -> Err + obj -> Intg);
    obj -> PIout = obj -> ff_SpeedRef +  (obj -> Err + obj -> Intg);

  //obj -> SpeedEst = _IQsat( obj -> PIout, obj -> SpeedEstMaxLim, (-obj -> SpeedEstMaxLim) );
    obj -> SpeedEst = MATH_sat( obj -> PIout, obj -> SpeedEstMaxLim, (-obj -> SpeedEstMaxLim) );

    obj -> aw = (obj -> PIout - obj -> SpeedEst);

    /* Calculate Estimated Electrical Angle */
  //obj -> ElecThetaEst += _IQmpy(obj -> AngIntgGain, obj -> SpeedEst);
    obj -> ElecThetaEst += (obj -> AngIntgGain * obj -> SpeedEst);

  //if (obj -> ElecThetaEst > _IQ(1.0))
    if (obj -> ElecThetaEst > (1.0f))
       {  obj -> ElecThetaEst -= (1.0f);}
    if (obj -> ElecThetaEst < (0.0f))
       {  obj -> ElecThetaEst += (1.0f);}

    /* Filter the Speed Estimation */
    /* 1st order Low-Pass Filter - Backward Euler discretization */
    obj -> SpeedEstFiltered += (obj -> LPF_SpeedEst_Gain * (obj -> Intg - obj -> SpeedEstFiltered));

    /* Injected High-Frequency Signal Generation */
  //obj -> InjVolAngle = (obj -> InjSampleCounter * obj -> InjVolAngleStep) + _IQdiv2(obj -> InjVolAngleStep);
    obj -> InjVolAngle = (obj -> InjSampleCounter * obj -> InjVolAngleStep) + (obj -> InjVolAngleStep / 2.0f);
    if (obj -> InjVolAngle > (1.0f))
       {  obj -> InjVolAngle -= (1.0f);}
    /* Demodulation angle */
    obj -> DemodAngle = (obj -> InjSampleCounter * obj -> InjVolAngleStep);
    if (obj -> DemodAngle > (1.0f))
       {  obj -> DemodAngle -= (1.0f);}
    /* Increase the counter and then reset it when it reaches sample size. */
    obj -> InjSampleCounter++;
    if ( obj -> InjSampleCounter == (2.0f * obj -> InjVolHalfPrdSampleNo) )
       {obj -> InjSampleCounter = 0.0f;}

    /* If 'obj -> Udc' is too low, division may cause overflow and amplify noise. Coordinate this part with the allowed min DC-link voltage. */
    obj -> InjVolMagn_UdcComp = obj -> InjVolMagn;
    if (obj -> Udc > obj -> UdcCompMinVol)
       {  obj -> InjVolMagn_UdcComp = (obj -> InjVolMagn / obj -> Udc);} /* Another option is to use close-loop regulation of injection current amplitude, maybe using Goertzel. */
    obj -> InjVol = (obj -> InjVolMagn_UdcComp * cosf(obj -> InjVolAngle * MATH_TWO_PI)) ;
    //obj->InjVol = (obj->InjVolMagn* cosf(obj->InjVolAngle_ZOHComp*MATH_TWO_PI))*vdc_ref/sqrtf(3.0f);



    /* Magnet Polarity Detection */
  //if (obj -> MagnPolarDetectFinished == FALSE)
    if (obj -> MagnPolarDetectFinished == 0.0f)
       {  obj -> CtrlCycleCounter++;

          if ((obj -> CtrlCycleCounter >  obj -> Wait4SettleHFI) && (obj -> MagnPolarDetectMode == 0.0f))
             {  obj -> MagnPolarDetectMode = 1.0f;
                obj -> CtrlCycleCounter = 0.0f;}

          if ((obj -> CtrlCycleCounter >  obj -> Wait4SettleCurrent) && (obj -> MagnPolarDetectMode > 0.0f))
             {  obj -> GoertzelEnable = 1.0f;}

          if (obj -> GoertzelEnable == 1.0f)
             {  obj -> gId_Now = (obj -> IdFbk * (2.0f * obj -> InjVolAngleStep)) + (obj -> gId_Past1 * obj -> gGain) - obj -> gId_Past2;
                obj -> gId_Past2 = obj -> gId_Past1;
                obj -> gId_Past1 = obj -> gId_Now;
                obj -> GoertzelCounter++;

                if (obj -> GoertzelCounter == 2.0f * obj -> InjVolHalfPrdSampleNo)
                   {  obj -> InjIdMagn = sqrtf( (obj -> gId_Past1 * obj -> gId_Past1) - ((obj -> gId_Past1 * obj -> gGain) * obj -> gId_Past2) + (obj -> gId_Past2 * obj -> gId_Past2) );

                      obj -> gId_Past1 = (0.0f);
                      obj -> gId_Past2 = (0.0f);
                      obj -> GoertzelCounter = 0.0f;
                      obj -> CtrlCycleCounter = 0.0f;
                      obj -> GoertzelEnable = 0.0f;

                      if (obj -> MagnPolarDetectMode == 1.0f)
                         {  obj -> IdMagn_MPD_Mode1 = obj -> InjIdMagn;
                            obj -> MagnPolarDetectMode = 2.0f;}
                      else if (obj -> MagnPolarDetectMode == 2.0f)
                         {  obj -> IdMagn_MPD_Mode2 = obj -> InjIdMagn;
                            obj -> MagnPolarDetectMode = 0.0f;

                            obj -> MagnPolarDetectFinished = 1.0f;

                            if (obj -> IdMagn_MPD_Mode1 > obj -> IdMagn_MPD_Mode2)
                               {  obj -> ElecThetaEst += (0.5f);}

                            if (obj -> ElecThetaEst > (1.0f))
                               {  obj -> ElecThetaEst -= (1.0f);}
                            if (obj -> ElecThetaEst < (0.0f))
                               {  obj -> ElecThetaEst += (1.0f);}
                          }
                     }
               }
       }


    return;
} // end of HFI_run_series() function

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
