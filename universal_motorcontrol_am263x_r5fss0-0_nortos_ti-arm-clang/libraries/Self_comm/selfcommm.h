//#############################################################################
//
// FILE:   pi.h
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

#ifndef SELFCOMMM_H
#define SELFCOMMM_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup PI PI
//! @{
//
//*****************************************************************************

//#include "types.h"
//#include "libraries/math/include/math.h"
//#include <math.h>

#include <stdlib.h>
#include <math.h>

#include "math_types.h"

//*****************************************************************************
//
//! brief Defines the PI controller object
//
//*****************************************************************************
typedef struct _SELFCOMMM_Obj_
{
    float32_t VariableInit;                    // Data: 1.0f/0.0f. Variable initialization is enabled.
    float32_t VoltageFound;                  // Data: 1.0f/0.0f. Magnitude of the injected voltage is found.
    float32_t VoltageNotFound;               // Data: 1.0f/0.0f. Magnitude of the injected voltage is not found with given min/max current and min frequency limit. Error Signal.
    float32_t VolMinAssigned;                // Data: 1.0f/0.0f. Min search value of the injected voltage magnitude is assigned.
    float32_t VolMaxAssigned;                // Data: 1.0f/0.0f. Max search value of the injected voltage magnitude is assigned.
    float32_t ExecuteEnable;                 // Data: 1.0f/0.0f. Enable calculations to detect current amplitude, select injection signal, and find inductance.
    float32_t InjVolHalfPrdSampleNo;         // Data: Number. Half of sample number of injected voltage in its one period - currently utilized value.
    float32_t InjSampleCounter;              // Data: Number. Counter for injected sample numbers of sinusoidal injection voltage.
    float32_t InjPrdCounter;                 // Data: Number. Counter for completed periods of injected sinusoidal voltage.
    float32_t FirstLdqEstCheck;              // Data: Number. Counter for inductance calculation execution.
    float32_t FirstVolMaxLimHitCheck;        // Data: Number. Counter for hitting the max injected voltage magnitude limit.
    float32_t FirstFreqMinLimHitCheck;       // Data: Number. Counter for hitting the min injected voltage frequency limit.
    float32_t InjVolAngleStep;                  // Data: Per-Unit. Angle steps to increment 'InjVolAngle'.
    float32_t InjVolMagn;                       // Data: Per-Unit. Injected voltage magnitude.
    float32_t InjVolFreq;                       // Data: Per-Unit. Injected voltage frequency.
    float32_t InjVolAngle;                      // Data: Per-Unit. Injected voltage cosine angle.
    float32_t InjVolAngle_ZOHComp;              // Data: Per-Unit. Injected voltage cosine angle, compensated for PWM's ZOH phase delay.
    float32_t ElecThetaUnsat;                   // Data: Per-Unit. Assigned reference frame electical angle, unsaturated.
    float32_t MinSearchVol;                     // Data: Per-Unit. Min search value of the injection voltage magnitude.
    float32_t MaxSearchVol;                     // Data: Per-Unit. Max search value of the injection voltage magnitude.
    float32_t InputGainDFT;                     // Data: Gain. The coefficient used to multiply the DFT input to prevent overflow and to find exact amplitude of the frequency component.
    float32_t CosDFT;                           // Data: Gain. Cosine value used in DFT computation.
    float32_t SinDFT;                           // Data: Gain. Sine value used in DFT computation.
    float32_t VdRealDFT;                        // Data: Per-Unit. Real part of the DFT of D-axis voltage.
    float32_t VdImagDFT;                        // Data: Per-Unit. Imaginary part of the DFT of D-axis voltage.
    float32_t IdRealDFT;                        // Data: Per-Unit. Real part of the DFT of D-axis current.
    float32_t IdImagDFT;                        // Data: Per-Unit. Imaginary part of the DFT of D-axis current.
    float32_t IqRealDFT;                        // Data: Per-Unit. Real part of the DFT of Q-axis current.
    float32_t IqImagDFT;                        // Data: Per-Unit. Imaginary part of the DFT of Q-axis current.
    float32_t VdMagnDFT;                        // Data: Per-Unit. D-axis voltage peak magnitude.
    float32_t IdMagnDFT;                        // Data: Per-Unit. D-axis current peak magnitude.
    float32_t IqMagnDFT;                        // Data: Per-Unit. Q-axis current peak magnitude.
    float32_t IsMagn;                           // Data: Per-Unit. Stator current vector peak magnitude.
    float32_t Lpu;                              // Data: Per-Unit. Calculated inductance at a given 'ElecTheta'.
    float32_t VdPhaseDFT;                       // Data: Per-Unit. D-axis voltage phase angle.
    float32_t IdPhaseDFT;                       // Data: Per-Unit. D-axis current phase angle.

    float32_t InjVolHalfPrdSampleNoMax;         // Parameter: Number. Allowed max value for injected voltage half period sample number i.e. minimum injection frequency.
    float32_t InjVolHalfPrdSampleNoInit;        // Parameter: Number. Half of sample number of injected voltage in its one period - initial value.
    float32_t InjSettlePrd;                     // Parameter: Number. Number of injected sinusoidal voltage periods to be completed for settling.
    float32_t AngIntgGain;                      // Parameter: Gain. Angle integration gain: Fbase*Tsample.
    float32_t ElecThetaStep;                    // Parameter: Per-Unit. Angle steps to increment 'ElecThetaUnsat' and hence 'ElecTheta'.
    float32_t InjVolMagnInit;                   // Parameter: Per-Unit. Initial value for injected voltage magnitude.
    float32_t IsMagnMinLim;                     // Parameter: Per-Unit. Allowed min value for conducted current magnitude.
    float32_t IsMagnMaxLim;                     // Parameter: Per-Unit. Allowed max value for conducted current magnitude.
    float32_t InjVolMagnMaxLim;                 // Parameter: Per-Unit. Allowed max value for injected voltage magnitude.
    float32_t Vdc_Base;                         // Parameter: DC_link Base value.
    float32_t Vph_Base;                         // Parameter: Phase voltage Base value.
    float32_t I_Base;                           // Parameter: DC_link Base value.
    float32_t F_Base;                           // Parameter: DC_link Base value.

    float32_t Udc;                              // Input: Per-Unit. DC-link voltage feedback.
    float32_t IdFbk;                            // Input: Per-Unit. Id current feedback.
    float32_t IqFbk;                            // Input: Per-Unit. Iq current feedback.

    float32_t InjVol;                           // Output: Per-Unit. Injected voltage reference for PWM block.
    float32_t ElecTheta;                        // Output: Per-Unit. Assigned reference frame electical angle.
    float32_t LminApprox;                       // Output: Per-Unit. Approximate min spatial inductance.
    float32_t LmaxApprox;                       // Output: Per-Unit. Approximate max spatial inductance.
    float32_t InitialRotorAngle;                // Output: Per-Unit. Approximate initial rotor electrical angle. But magnet polarity is to be determined later.
    float32_t Finish;                        // Output: 1.0f/0.0f. Procedure is finished.

    float32_t Ind_Total[500];
    float32_t Ang_Total[500];
    uint16_t i12;

} SELFCOMMM_Obj;

//*****************************************************************************
//
//! brief Defines the PI handle
//
//*****************************************************************************
typedef struct _SELFCOMMM_Obj_ *SELFCOMMM_Handle;

//*****************************************************************************
//
//! brief     Gets the feedback value in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! return    The feedback value in the PI controller
//
//*****************************************************************************
//static inline float32_t
//PI_getFbackValue(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->fbackValue);
//} // end of PI_getFbackValue() function

//*****************************************************************************
//
//! brief     Gets the feedforward value in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! return    The feedforward value in the PI controller
//
//*****************************************************************************
//static inline float32_t
//PI_getFfwdValue(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->ffwdValue);
//} // end of PI_getFfwdValue() function

//*****************************************************************************
//
//! brief      Gets the gains in the PI controller
//!
//! param[in]  handle  The PI controller handle
//!
//! param[out] pKp     The pointer to the proportional gain value
//!
//! param[out] pKi     The pointer to the integrator gain value
//!
//! return     None
//
//*****************************************************************************
//static inline void
//PI_getGains(PI_Handle handle, float32_t *pKp, float32_t *pKi)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    *pKp = obj->Kp;
//    *pKi = obj->Ki;
//
//    return;
//} // end of PI_getGains() function

//*****************************************************************************
//
//! brief     Gets the integral gain in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! return    The integral gain in the PI controller
//
//*****************************************************************************
//static inline float32_t
//PI_getKi(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->Ki);
//} // end of PI_getKi() function

//*****************************************************************************
//
//! brief     Gets the proportional gain in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! return    The proportional gain in the PI controller
//
//*****************************************************************************
//static inline float32_t
//PI_getKp(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->Kp);
//} // end of PI_getKp() function

//*****************************************************************************
//
//! brief      Gets the minimum and maximum output value allowed in the PI
//!             controller
//!
//! param[in]  handle   The PI controller handle
//!
//! param[out] pOutMin  The pointer to the minimum output value allowed
//!
//! param[out] pOutMax  The pointer to the maximum output value allowed
//!
//! return     None
//
//*****************************************************************************
//static inline void
//PI_getMinMax(PI_Handle handle, float32_t *pOutMin, float32_t *pOutMax)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    *pOutMin = obj->outMin;
//    *pOutMax = obj->outMax;
//
//    return;
//} // end of PI_getMinMax() function

//*****************************************************************************
//
//! brief      Gets the maximum output value allowed in the PI controller
//!
//! param[in]  handle  The PI controller handle
//!
//! return     The maximum output value allowed
//
//*****************************************************************************
//static inline float32_t
//PI_getOutMax(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->outMax);
//} // end of PI_getOutMax() function

//*****************************************************************************
//
//! brief      Gets the minimum output value allowed in the PI controller
//!
//! param[in]  handle  The PI controller handle
//!
//! return     The minimum output value allowed
//
//*****************************************************************************
//static inline float32_t
//PI_getOutMin(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->outMin);
//} // end of PI_getOutMin() function

//*****************************************************************************
//
//! brief     Gets the reference value in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! return    The reference value in the PI controller
//
//*****************************************************************************
//static inline float32_t
//PI_getRefValue(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->refValue);
//} // end of PI_getRefValue() function

//*****************************************************************************
//
//! brief     Gets the integrator start value in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! return    The integrator start value for the PI controller
//
//*****************************************************************************
//static inline float32_t
//PI_getUi(PI_Handle handle)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    return(obj->Ui);
//} // end of PI_getUi() function

//*****************************************************************************
//
//! brief     Initializes the PI controller
//!
//! param[in] pMemory   A pointer to the memory for the PI controller object
//!
//! param[in] numBytes  The number of bytes allocated for the PI controller
//!                      object, bytes
//!
//! return    The PI controller (PI) object handle
//
//*****************************************************************************
extern SELFCOMMM_Handle SELFCOMMM_init(void *pMemory, const size_t numBytes);

extern void
SELFCOMM_setParams(SELFCOMMM_Handle handle);


//extern SELFCOMM_Handle
//cla_PI_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! brief     Sets the feedback value in the PI controller
//!
//! param[in] handle      The PI controller handle
//!
//! param[in] fbackValue  The feedback value
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setFbackValue(PI_Handle handle, const float32_t fbackValue)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->fbackValue = fbackValue;
//
//    return;
//} // end of PI_setFbackValue() function

//*****************************************************************************
//
//! brief     Sets the feedforward value in the PI controller
//!
//! param[in] handle     The PI controller handle
//!
//! param[in] ffwdValue  The feedforward value
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setFfwdValue(PI_Handle handle, const float32_t ffwdValue)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->ffwdValue = ffwdValue;
//
//    return;
//} // end of PI_setFfwdValue() function

//*****************************************************************************
//
//! brief     Sets the gains in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] Kp      The proportional gain for the PI controller
//!
//! param[in] Ki      The integrator gain for the PI controller
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setGains(PI_Handle handle, const float32_t Kp, const float32_t Ki)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->Kp = Kp;
//    obj->Ki = Ki;
//
//    return;
//} // end of PI_setGains() function

//*****************************************************************************
//
//! brief     Sets the integral gain in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] Ki         The integral gain for the PI controller
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setKi(PI_Handle handle, const float32_t Ki)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->Ki = Ki;
//
//    return;
//} // end of PI_setKi() function

//*****************************************************************************
//
//! brief     Sets the proportional gain in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] Kp         The proportional gain for the PI controller
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setKp(PI_Handle handle, const float32_t Kp)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->Kp = Kp;
//
//    return;
//} // end of PI_setKp() function

//*****************************************************************************
//
//! brief     Sets the minimum and maximum output value allowed in the PI
//!            controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] outMin  The minimum output value allowed
//!
//! param[in] outMax  The maximum output value allowed
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setMinMax(PI_Handle handle, const float32_t outMin, const float32_t outMax)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->outMin = outMin;
//    obj->outMax = outMax;
//
//    return;
//} // end of PI_setMinMax() function

//*****************************************************************************
//
//! brief     Sets the maximum output value allowed in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] outMax  The maximum output value allowed
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setOutMax(PI_Handle handle, const float32_t outMax)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->outMax = outMax;
//
//    return;
//} // end of PI_setOutMax() function

//*****************************************************************************
//
//! brief     Sets the minimum output value allowed in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] outMax  The minimum output value allowed
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setOutMin(PI_Handle handle, const float32_t outMin)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->outMin = outMin;
//
//    return;
//} // end of PI_setOutMin() function

//*****************************************************************************
//
//! brief     Sets the reference value in the PI controller
//!
//! param[in] handle    The PI controller handle
//!
//! param[in] refValue  The reference value
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setRefValue(PI_Handle handle, const float32_t refValue)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->refValue = refValue;
//
//    return;
//} // end of PI_setRefValue() function

//*****************************************************************************
//
//! brief     Sets the integrator start value in the PI controller
//!
//! param[in] handle  The PI controller handle
//!
//! param[in] Ui      The integral start value for the PI controller
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_setUi(PI_Handle handle, const float32_t Ui)
//{
//    PI_Obj *obj = (PI_Obj *)handle;
//
//    obj->Ui = Ui;
//
//    return;
//} // end of PI_setUi() function

//*****************************************************************************
//
//! brief     Runs the parallel form of the PI controller
//!
//! param[in] handle      The PI controller handle
//!
//! param[in] refValue    The reference value to the controller
//!
//! param[in] fbackValue  The feedback value to the controller
//!
//! param[in] ffwdValue   The feedforward value to the controller
//!
//! param[in] pOutValue   The pointer to the controller output value
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_run_parallel(PI_Handle handle, const float32_t refValue,
//                const float32_t fbackValue, const float32_t ffwdValue,
//                float32_t *pOutValue)
//{
//    float32_t Error;
//    float32_t Kp = PI_getKp(handle);
//    float32_t Ki = PI_getKi(handle);
//    float32_t Up;
//    float32_t Ui = PI_getUi(handle);
//    float32_t outMax = PI_getOutMax(handle);
//    float32_t outMin = PI_getOutMin(handle);
//
//    Error = refValue - fbackValue;
//
//    //
//    // Compute the proportional output
//    //
//    Up = Kp * Error;
//
//    //
//    // Compute the integral output
//    //
//    Ui = MATH_sat(Ui + (Ki * Error),outMax,outMin);
//
//    PI_setUi(handle,Ui);
//    PI_setRefValue(handle,refValue);
//    PI_setFbackValue(handle,fbackValue);
//    PI_setFfwdValue(handle,ffwdValue);
//
//    //
//    // Saturate the output
//    //
//    *pOutValue = MATH_sat(Up + Ui + ffwdValue,outMax,outMin);
//
//    return;
//} // end of PI_run_parallel() function

//*****************************************************************************
//
//! brief     Runs the series form of the PI controller
//!
//! param[in] handle      The PI controller handle
//!
//! param[in] refValue    The reference value to the controller
//!
//! param[in] fbackValue  The feedback value to the controller
//!
//! param[in] ffwdValue   The feedforward value to the controller
//!
//! param[in] pOutValue   The pointer to the controller output value
//!
//! return    None
//
//*****************************************************************************
//static inline void
//PI_run_series(PI_Handle handle, const float32_t refValue,
//              const float32_t fbackValue, const float32_t ffwdValue,
//              float32_t *pOutValue)
//{
//    float32_t Error;
//    float32_t Kp = PI_getKp(handle);
//    float32_t Ki = PI_getKi(handle);
//    float32_t Up;
//    float32_t Ui = PI_getUi(handle);
//    float32_t outMax = PI_getOutMax(handle);
//    float32_t outMin = PI_getOutMin(handle);
//
//    Error = refValue - fbackValue;
//
//    // Compute the proportional output
//    Up = Kp * Error;
//
//    // Compute the integral output with saturation
//#ifdef __TMS320C28XX_CLA__
//    Ui = MATH_sat(Ui + (Ki * Up), outMax, outMin);
//#else
//    Ui = MATH_sat(Ui + (Ki * Up), outMax, outMin);
//#endif  // __TMS320C28XX_CLA__
//
//    PI_setUi(handle,Ui);
//    PI_setRefValue(handle, refValue);
//    PI_setFbackValue(handle, fbackValue);
//    PI_setFfwdValue(handle, ffwdValue);
//
//    // Saturate the output
//#ifdef __TMS320C28XX_CLA__
//    *pOutValue = MATH_sat(Up + Ui + ffwdValue, outMax, outMin);
//#else
//    *pOutValue = MATH_sat(Up + Ui + ffwdValue, outMax, outMin);
//#endif  // __TMS320C28XX_CLA__
//
//    return;
//} // end of PI_run_series() function


//*****************************************************************************
//
//! brief     Runs the series form of the PI controller
//!
//! param[in] handle      The PI controller handle
//!
//! param[in] refValue    The reference value to the controller
//!
//! param[in] fbackValue  The feedback value to the controller
//!
//! param[in] pOutValue   The pointer to the controller output value
//!
//! return    None
//
//*****************************************************************************

//static inline void
//SELFCOMMM_run(SELFCOMMM_Handle handle, const float32_t vdc_ref,
//       const float32_t id_fb, const float32_t iq_fb, float32_t *pOutValue1, float32_t *pOutValue2, float32_t *pOutValue3)
static inline void
SELFCOMMM_run(SELFCOMMM_Handle handle, const float32_t vdc_ref,
       const float32_t id_fb, const float32_t iq_fb)
{

    SELFCOMMM_Obj *obj = (SELFCOMMM_Obj *)handle;

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
