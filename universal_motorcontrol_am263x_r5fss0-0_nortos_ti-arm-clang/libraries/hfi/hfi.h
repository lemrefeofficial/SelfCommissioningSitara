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

#define MtrType_INDUCTION               (1)
#define MtrType_IPMSM                   (2)
#define MtrType_SYNRM                   (3)

/*************************************************************************************************************************************************************************************/
// Defines the HFI object
/*************************************************************************************************************************************************************************************/
typedef struct _HFI_Obj_
{
    float32_t VariableInit;                 // Data: TRUE/FALSE. Variable initialization is enabled.
    float32_t SignalSelectionError;          // Data: TRUE/FALSE. Injection signal cannot be selected under the limitation of min injection frequency and target current amplitude.
    float32_t GoertzelEnable;                // Data: TRUE/FALSE. High-frequency current amplitude measurement algorithm for initial postion detection is enabled.
    float32_t MagnPolarDetectFinished;       // Data: TRUE/FALSE. Magnet polarity is detected. Initial postion detection is achieved.
    float32_t MagnPolarDetectMode;           // Data: Number. Meaning 0: Idle mode, 1: Negative Id, 2: Positive Id
    float32_t CtrlCycleCounter;              // Data: Number. Counter for ctrl entry in this module, used for timing in initial rotor postion detection.
    float32_t InjSampleCounter;              // Data: Number. Counter for injected sample numbers of sinusoidal injection voltage.
    float32_t GoertzelCounter;               // Data: Number. Counter for samples used in High-frequency current amplitude measurement algorithm.
    float32_t Wait4SettleHFI;                // Data: Number. Samples to wait for HFI algorithm to settle.
    float32_t Wait4SettleCurrent;            // Data: Number. Samples to wait for current loop to settle.
    float32_t InjVolHalfPrdSampleNo;         // Data: Number. Half of sample number of injected voltage in its one period - currently utilized value.
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

    float32_t InjVolHalfPrdSampleNoInit;     // Parameter: Number. Half of sample number of injected voltage in its one period - initial value.
    float32_t InjVolHalfPrdSampleNoMaxLim;   // Parameter: Number. Allowed max value for injected voltage half period sample number i.e. minimum injection frequency.
    float32_t SampleNoBase;                  // Parameter: Number. Sample number in one base period: Fsample/Fbase.
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

static inline void HFI_run(HFI_Handle handle, const float32_t MechSpeed, const float32_t iq_fb, const float32_t Vq_fb)
{

    HFI_Obj *obj = (HFI_Obj *)handle;

        /* Initialize some variables. */
        /* This enables safe start and usage of this routine again without resetting CPU. */

    if (obj -> VariableInit == TRUE)                                                                                                                                                                     \
       {  obj -> VariableInit = FALSE;                                                                                                                                                                   \
                                                                                                                                                                                                    \
          obj -> SignalSelectionError = FALSE;                                                                                                                                                           \
          obj -> GoertzelEnable = FALSE;                                                                                                                                                                 \
          obj -> MagnPolarDetectFinished = FALSE;                                                                                                                                                        \
          obj -> MagnPolarDetectMode = 0;                                                                                                                                                                \
          obj -> CtrlCycleCounter = 0;                                                                                                                                                                   \
          obj -> InjSampleCounter = 0;                                                                                                                                                                   \
          obj -> GoertzelCounter = 0;                                                                                                                                                                    \
                                                                                                                                                                                                    \
          obj -> InjVolHalfPrdSampleNo = obj -> InjVolHalfPrdSampleNoInit - 1;                                                                                                                                \
          do {  obj -> InjVolHalfPrdSampleNo++;                                                                                                                                                          \
                if (obj -> InjVolHalfPrdSampleNo > obj -> InjVolHalfPrdSampleNoMaxLim)                                                                                                                        \
                   {  obj -> InjVolHalfPrdSampleNo = obj -> InjVolHalfPrdSampleNoMaxLim;                                                                                                                      \
                      obj -> InjVolMagnMaxLim = obj -> InjVolMagn;    /* Precaution to break the possible endless loop. */                                                                                    \
                      obj -> SignalSelectionError = TRUE; /* DONT FORGET TO USE THIS ERROR MESSAGE. */                                                                                                   \
                   }                                                                                                                                                                                \
                                                                                                                                                                                                    \
                /* '_IQdiv' can work for the division of two integer variables. */                                                                                                                  \
                obj -> InjVolAngleStep = _IQdiv( (1UL), (2*obj -> InjVolHalfPrdSampleNo) );                                                                                                                   \
                obj -> InjVolFreq = _IQdiv(obj -> InjVolAngleStep, obj -> AngIntgGain);                                                                                                                            \
                                                                                                                                                                                                    \
                obj -> InjVolMagn = _IQmpy(_IQmpy(obj -> InjVolFreq, obj -> Ld), obj -> IhfiTarget);                                                                                                                    \
             }                                                                                                                                                                                      \
          while (obj -> InjVolMagn > obj -> InjVolMagnMaxLim);                                                                                                                                                \
                                                                                                                                                                                                    \
          obj -> Kerror = _IQdiv( _IQmpy(obj -> InjVolMagn, _IQdiv2(obj -> Lq - obj -> Ld)), _IQmpy(obj -> InjVolFreq, _IQmpy(obj -> Lq, obj -> Ld)) );                                                                                \
          obj -> HFI_Wc = _IQdiv(obj -> InjVolFreq, obj -> Ratio_InjFreq_WcHFI);                                                                                                                                   \
                                                                                                                                                                                                    \
          /* OVERFLOW TEST FOR FIXED-POINT IMPLEMENTATION: Decrease target 'obj -> HFI_Wc'. */                                                                                                           \
          if (MATH_TYPE == IQ_MATH)                                                                                                                                                                 \
             {  obj -> Kp_hfi = _IQdiv(obj -> Kerror, obj -> HFI_Wc); /* In reality, it is 1/Kp which is small and hence safe. */                                                                                  \
                while ( obj -> Kp_hfi < _IQ( 1.0/( 1UL<<(30 - GLOBAL_Q) ) ) )                                                                                                                            \
                    {  obj -> Kp_hfi = _IQmpy2(obj -> Kp_hfi);                                                                                                                                                \
                       obj -> HFI_Wc = _IQdiv2(obj -> HFI_Wc);                                                                                                                                                \
                       obj -> Ratio_InjFreq_WcHFI = _IQmpy2(obj -> Ratio_InjFreq_WcHFI);} /* Update the ratio which is now utilized. */                                                                       \
             }                                                                                                                                                                                      \
                                                                                                                                                                                                    \
          obj -> Kp_hfi = _IQdiv(obj -> HFI_Wc, obj -> Kerror); /* CHECKED FOR OVERFLOW. */                                                                                                                        \
          obj -> Ki_hfi = _IQmpy(obj -> TimePU, _IQdiv(obj -> HFI_Wc, obj -> Ratio_WcHFI_WcLPF));                                                                                                                       \
                                                                                                                                                                                                    \
          obj -> LPF_Error_Wc = _IQmpy(obj -> HFI_Wc, obj -> Ratio_WcHFI_WcLPF);                                                                                                                                   \
          obj -> LPF_SpeedEst_Wc = obj -> HFI_Wc;                                                                                                                                                             \
                                                                                                                                                                                                    \
          /* Current loop need 1.5 multiples and position estimator needs 2 multiples. But it is secure to use 3 as multiplier. */                                                                  \
          obj -> Wait4SettleCurrent = 3*_IQmpy((_iq)obj -> SampleNoBase, _IQdiv(_IQ(1.0), _IQmpy(obj -> Ratio_WcCurrent_WcHFI, obj -> HFI_Wc)));                                                                        \
          obj -> Wait4SettleHFI = 3*_IQmpy((_iq)obj -> SampleNoBase, _IQdiv(_IQ(1.0), obj -> HFI_Wc));                                                                                                             \
                                                                                                                                                                                                    \
          obj -> Intg = _IQ(0.0);                                                                                                                                                                        \
          obj -> aw = _IQ(0.0);                                                                                                                                                                          \
                                                                                                                                                                                                    \
          obj -> InjVolAngle = -_IQdiv2(obj -> InjVolAngleStep); /* Injection angle starts half sample positive to consider ZOH nature of the inverter. */                                                    \
          obj -> DemodAngle = -obj -> InjVolAngleStep; /* Start one sample shifted to consider the computation delay. */                                                                                      \
          obj -> BPF_Id_OutPast1 = _IQ(0.0);                                                                                                                                                             \
          obj -> BPF_Id_OutPast2 = _IQ(0.0);                                                                                                                                                             \
          obj -> BPF_Id_InPast1 = _IQ(0.0);                                                                                                                                                              \
          obj -> BPF_Id_InPast2 = _IQ(0.0);                                                                                                                                                              \
          obj -> BPF_Iq_OutPast1 = _IQ(0.0);                                                                                                                                                             \
          obj -> BPF_Iq_OutPast2 = _IQ(0.0);                                                                                                                                                             \
          obj -> BPF_Iq_InPast1 = _IQ(0.0);                                                                                                                                                              \
          obj -> BPF_Iq_InPast2 = _IQ(0.0);                                                                                                                                                              \
          /* Filter design. */                                                                                                                                                                      \
          obj -> BPF_GainOut1 = _IQdiv( _IQmpy2(_IQcosPU(obj -> InjVolAngleStep)), (_IQ(1.0) + _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep))) );                                                            \
          obj -> BPF_GainOut2 = _IQdiv( ( _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) - _IQ(1.0) ), ( _IQ(1.0) + _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) ) );                                \
          obj -> BPF_GainIn = _IQdiv( ( _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) ), ( _IQ(1.0) + _IQmpy(obj -> BPF_Zeta, _IQsinPU(obj -> InjVolAngleStep)) ) );                                             \
          obj -> LPF_Error_Gain = _IQdiv(_IQmpy(obj -> LPF_Error_Wc, obj -> TimePU), (_IQ(1.0) + _IQmpy(obj -> LPF_Error_Wc, obj -> TimePU)));                                                                               \
          obj -> LPF_SpeedEst_Gain = _IQdiv(_IQmpy(obj -> LPF_SpeedEst_Wc, obj -> TimePU), (_IQ(1.0) + _IQmpy(obj -> LPF_SpeedEst_Wc, obj -> TimePU)));                                                                      \
          /* Goertzel variables. */                                                                                                                                                                 \
          obj -> gGain = _IQmpy2(_IQcosPU(obj -> InjVolAngleStep));                                                                                                                                           \
          obj -> gId_Past1 = _IQ(0.0);                                                                                                                                                                   \
          obj -> gId_Past2 = _IQ(0.0);                                                                                                                                                                   \
                                                                                                                                                                                                    \
          obj -> SpeedEstFiltered = _IQ(0.0);                                                                                                                                                            \
       }

    /* Extract the harmonic current in the Estimated Q-axis */                                                                                                                                     \
    /* 2nd order Band-Pass Filter - Direct Form 1 */                                                                                                                                                \
    /* Tustin discretization - Prewarped at the BPF center freqency for zero phase lag there */                                                                                                     \
    obj -> BPF_Iq_Out = _IQmpy(obj -> BPF_GainIn, (obj -> IqFbk - obj -> BPF_Iq_InPast2)) + _IQmpy(obj -> BPF_GainOut1, obj -> BPF_Iq_OutPast1) + _IQmpy(obj -> BPF_GainOut2, obj -> BPF_Iq_OutPast2);                                      \
                                                                                                                                                                                                    \
    obj -> BPF_Iq_OutPast2 = obj -> BPF_Iq_OutPast1;                                                                                                                                                          \
    obj -> BPF_Iq_OutPast1 = obj -> BPF_Iq_Out;                                                                                                                                                               \
    obj -> BPF_Iq_InPast2 = obj -> BPF_Iq_InPast1;                                                                                                                                                            \
    obj -> BPF_Iq_InPast1 = obj -> IqFbk;                                                                                                                                                                     \
                                                                                                                                                                                                    \
                                                                                                                                                                                                    \
    obj -> BPF_Id_Out = _IQmpy(obj -> BPF_GainIn, (obj -> IdFbk - obj -> BPF_Id_InPast2)) + _IQmpy(obj -> BPF_GainOut1, obj -> BPF_Id_OutPast1) + _IQmpy(obj -> BPF_GainOut2, obj -> BPF_Id_OutPast2);                                      \
                                                                                                                                                                                                    \
    obj -> BPF_Id_OutPast2 = obj -> BPF_Id_OutPast1;                                                                                                                                                          \
    obj -> BPF_Id_OutPast1 = obj -> BPF_Id_Out;                                                                                                                                                               \
    obj -> BPF_Id_InPast2 = obj -> BPF_Id_InPast1;                                                                                                                                                            \
    obj -> BPF_Id_InPast1 = obj -> IdFbk;                                                                                                                                                                     \
                                                                                                                                                                                                    \
    /* By Low-pass Filtering, Extract the DC Error due to Estimated and Actual Axis Mismatch  */                                                                                                    \
    /* 1st order Low-Pass Filter - Backward Euler discretization */                                                                                                                                 \
    obj -> LPF_Error_Out += _IQmpy(obj -> LPF_Error_Gain, (_IQmpy(obj -> BPF_Iq_Out, _IQsinPU(obj -> DemodAngle)) - obj -> LPF_Error_Out));                                                                                  \
                                                                                                                                                                                                    \
    /* PI Controller */                                                                                                                                                                             \
    obj -> Err = _IQmpy(obj -> Kp_hfi, obj -> LPF_Error_Out);                                                                                                                                                      \
    obj -> Intg += _IQmpy(obj -> Ki_hfi, (obj -> Err - obj -> aw));                                                                                                                                                     \
    obj -> PIout = obj -> ff_SpeedRef +  (obj -> Err + obj -> Intg);                                                                                                                                                    \
    obj -> SpeedEst = _IQsat( obj -> PIout, obj -> SpeedEstMaxLim, (-obj -> SpeedEstMaxLim) );                                                                                                                          \
    obj -> aw = (obj -> PIout - obj -> SpeedEst);                                                                                                                                                                  \
                                                                                                                                                                                                    \
    /* Calculate Estimated Electrical Angle */                                                                                                                                                      \
    obj -> ElecThetaEst += _IQmpy(obj -> AngIntgGain, obj -> SpeedEst);                                                                                                                                            \
    if (obj -> ElecThetaEst > _IQ(1.0))                                                                                                                                                                  \
       {  obj -> ElecThetaEst -= _IQ(1.0);}                                                                                                                                                              \
    if (obj -> ElecThetaEst < _IQ(0.0))                                                                                                                                                                  \
       {  obj -> ElecThetaEst += _IQ(1.0);}                                                                                                                                                              \
                                                                                                                                                                                                    \
    /* Filter the Speed Estimation */                                                                                                                                                               \
    /* 1st order Low-Pass Filter - Backward Euler discretization */                                                                                                                                 \
    obj -> SpeedEstFiltered += _IQmpy(obj -> LPF_SpeedEst_Gain, (obj -> Intg - obj -> SpeedEstFiltered));                                                                                                               \
                                                                                                                                                                                                    \
    /* Injected High-Frequency Signal Generation */                                                                                                                                                 \
    obj -> InjVolAngle = (obj -> InjSampleCounter * obj -> InjVolAngleStep) + _IQdiv2(obj -> InjVolAngleStep);                                                                                                          \
    if (obj -> InjVolAngle > _IQ(1.0))                                                                                                                                                                   \
       {  obj -> InjVolAngle -= _IQ(1.0);}                                                                                                                                                               \
    /* Demodulation angle */                                                                                                                                                                        \
    obj -> DemodAngle = (obj -> InjSampleCounter * obj -> InjVolAngleStep);                                                                                                                                        \
    if (obj -> DemodAngle > _IQ(1.0))                                                                                                                                                                    \
       {  obj -> DemodAngle -= _IQ(1.0);}                                                                                                                                                                \
    /* Increase the counter and then reset it when it reaches sample size. */                                                                                                                       \
    obj -> InjSampleCounter++;                                                                                                                                                                           \
    if ( obj -> InjSampleCounter == (2*obj -> InjVolHalfPrdSampleNo) )                                                                                                                                        \
       {obj -> InjSampleCounter = 0;}                                                                                                                                                                    \
                                                                                                                                                                                                    \
    /* If 'obj -> Udc' is too low, division may cause overflow and amplify noise. Coordinate this part with the allowed min DC-link voltage. */                                                          \
    obj -> InjVolMagn_UdcComp = obj -> InjVolMagn;                                                                                                                                                            \
    if (obj -> Udc > obj -> UdcCompMinVol)                                                                                                                                                                    \
       {  obj -> InjVolMagn_UdcComp = _IQdiv(obj -> InjVolMagn, obj -> Udc);} /* Another option is to use close-loop regulation of injection current amplitude, maybe using Goertzel. */                           \
    obj -> InjVol = _IQmpy(obj -> InjVolMagn_UdcComp, _IQcosPU(obj -> InjVolAngle));                                                                                                                               \
                                                                                                                                                                                                    \
                                                                                                                                                                                                    \
                                                                                                                                                                                                    \
    /* Magnet Polarity Detection */                                                                                                                                                                 \
    if (obj -> MagnPolarDetectFinished == FALSE)                                                                                                                                                         \
       {  obj -> CtrlCycleCounter++;                                                                                                                                                                     \
                                                                                                                                                                                                    \
          if ((obj -> CtrlCycleCounter >  obj -> Wait4SettleHFI) && (obj -> MagnPolarDetectMode == 0))                                                                                                             \
             {  obj -> MagnPolarDetectMode = 1;                                                                                                                                                          \
                obj -> CtrlCycleCounter = 0;}                                                                                                                                                            \
                                                                                                                                                                                                    \
          if ((obj -> CtrlCycleCounter >  obj -> Wait4SettleCurrent) && (obj -> MagnPolarDetectMode > 0))                                                                                                          \
             {  obj -> GoertzelEnable = TRUE;}                                                                                                                                                           \
                                                                                                                                                                                                    \
          if (obj -> GoertzelEnable == TRUE)                                                                                                                                                             \
             {  obj -> gId_Now = _IQmpy(obj -> IdFbk, _IQmpy2(obj -> InjVolAngleStep)) + _IQmpy(obj -> gId_Past1, obj -> gGain) - obj -> gId_Past2;                                                                               \
                obj -> gId_Past2 = obj -> gId_Past1;                                                                                                                                                          \
                obj -> gId_Past1 = obj -> gId_Now;                                                                                                                                                            \
                obj -> GoertzelCounter++;                                                                                                                                                                \
                                                                                                                                                                                                    \
                if (obj -> GoertzelCounter == 2*obj -> InjVolHalfPrdSampleNo)                                                                                                                                 \
                   {  obj -> InjIdMagn = _IQsqrt( _IQmpy(obj -> gId_Past1, obj -> gId_Past1) - _IQmpy(_IQmpy(obj -> gId_Past1, obj -> gGain), obj -> gId_Past2) + _IQmpy(obj -> gId_Past2, obj -> gId_Past2) );                             \
                                                                                                                                                                                                    \
                      obj -> gId_Past1 = _IQ(0.0);                                                                                                                                                       \
                      obj -> gId_Past2 = _IQ(0.0);                                                                                                                                                       \
                      obj -> GoertzelCounter = 0;                                                                                                                                                        \
                      obj -> CtrlCycleCounter = 0;                                                                                                                                                       \
                      obj -> GoertzelEnable = FALSE;                                                                                                                                                     \
                                                                                                                                                                                                    \
                      if (obj -> MagnPolarDetectMode == 1)                                                                                                                                               \
                         {  obj -> IdMagn_MPD_Mode1 = obj -> InjIdMagn;                                                                                                                                       \
                            obj -> MagnPolarDetectMode = 2;}                                                                                                                                             \
                      else if (obj -> MagnPolarDetectMode == 2)                                                                                                                                          \
                         {  obj -> IdMagn_MPD_Mode2 = obj -> InjIdMagn;                                                                                                                                       \
                            obj -> MagnPolarDetectMode = 0;                                                                                                                                              \
                                                                                                                                                                                                    \
                            obj -> MagnPolarDetectFinished = TRUE;                                                                                                                                       \
                                                                                                                                                                                                    \
                            if (obj -> IdMagn_MPD_Mode1 > obj -> IdMagn_MPD_Mode2)                                                                                                                            \
                               {  obj -> ElecThetaEst += _IQ(0.5);}                                                                                                                                      \
                                                                                                                                                                                                    \
                            if (obj -> ElecThetaEst > _IQ(1.0))                                                                                                                                          \
                               {  obj -> ElecThetaEst -= _IQ(1.0);}                                                                                                                                      \
                            if (obj -> ElecThetaEst < _IQ(0.0))                                                                                                                                          \
                               {  obj -> ElecThetaEst += _IQ(1.0);}                                                                                                                                      \
                          }                                                                                                                                                                         \
                     }                                                                                                                                                                              \
               }                                                                                                                                                                                    \
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
