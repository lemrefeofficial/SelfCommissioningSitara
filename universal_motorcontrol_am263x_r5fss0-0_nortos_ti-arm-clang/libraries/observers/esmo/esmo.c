//#############################################################################
// $Copyright:
// Copyright (C) 2017-2023 Texas Instruments Incorporated - http://www.ti.com/
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

//! \file   \libraries\observers\esmo\source\esmo.c
//! \brief  Portable C floating-point code.  These functions define the
//!         enhanced smo estimator
//!

// **************************************************************************
// the includes
#include "esmo.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

ESMO_Handle ESMO_init(void *pMemory, const size_t numBytes)
{
    ESMO_Handle handle;

    if(numBytes < sizeof(ESMO_Obj))
    {
        return((ESMO_Handle)0x0);
    }

    // assign the handle
    handle = (ESMO_Handle)pMemory;

    return(handle);
}   // end of ESMO_init() function

//------------------------------------------------------------------------------
void ESMO_resetParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_ui = 0.0f;

    obj->speedEst = 0.0f;
    obj->theta = 0.0f;

    obj->Kslide = obj->KslideMin;
    obj->pll_Kp = obj->pll_KpMin;

    return;
}

//------------------------------------------------------------------------------
void ESMO_setPLLParams(ESMO_Handle handle, const float32_t pll_KpMax,
                       const float32_t pll_KpMin, const float32_t pll_KpSF)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_KpMax = pll_KpMax;
    obj->pll_KpMin = pll_KpMin;
    obj->pll_KpSF = pll_KpSF;

    return;
}

void ESMO_setPLLKi(ESMO_Handle handle, const float32_t pll_Ki)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_Ki = pll_Ki;

    return;
}

//------------------------------------------------------------------------------
void ESMO_setKslideParams(ESMO_Handle handle,
                          const float32_t KslideMax, const float32_t KslideMin)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->KslideMax = KslideMax;
    obj->KslideMin = KslideMin;

    return;
}

//------------------------------------------------------------------------------
void ESMO_setParams(ESMO_Handle handle, const USER_Params *pUserParams)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->scaleFreq_Hz = pUserParams->maxFrequency_Hz;

    obj->speed_sf = 1.0f / pUserParams->maxFrequency_Hz;
    obj->voltage_sf = (MATH_ONE_OVER_THREE / 4096.0f) / pUserParams->voltage_sf;
    obj->current_sf = (1.0f / 4096.0f) / pUserParams->current_sf;

    obj->Ts = pUserParams->ctrlPeriod_sec;
    obj->base_wTs = obj->Ts * obj->scaleFreq_Hz;
    obj->thetaDelta = obj->scaleFreq_Hz * obj->Ts;
    obj->thetaErrSF = MATH_ONE_OVER_TWO_PI;

    obj->Kslide = obj->KslideMin;

    obj->pll_Kp = obj->pll_KpMin;
    obj->pll_ui   = 0.0f;
    obj->pll_Ki   = 0.0f;
    obj->pll_Umax = 1.0f;       // The maximum value in Per-unit format, don't change
    obj->pll_Umin = -1.0f;      // The minimum value in Per-unit format, don't change

    obj->EstIalpha = 0.0f;
    obj->EstIbeta = 0.0f;

    obj->Zalpha = 0.0f;
    obj->Zbeta = 0.0f;

    obj->Ealpha = 0.0f;
    obj->Ebeta = 0.0f;
    float32_t motor_Rs = pUserParams->motor_Rs_Ohm;

    float32_t baseVIs = (pUserParams->voltage_sf * obj->Ts) / pUserParams->current_sf;

    obj->Fdsmopos = 1.0f - (motor_Rs / pUserParams->motor_Ls_d_H * obj->Ts);
    obj->Fqsmopos = 1.0f - (motor_Rs / pUserParams->motor_Ls_q_H * obj->Ts);
    obj->Gdsmopos = baseVIs / pUserParams->motor_Ls_d_H;
    obj->Gqsmopos = baseVIs / pUserParams->motor_Ls_q_H;


    ESMO_updateFilterParams(handle);

    return;
}

//------------------------------------------------------------------------------
void ESMO_updateFilterParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    // TempVarLpf = Fc *2 * PI * Ts
    float32_t tempVarLpf = obj->lpfFc_Hz * MATH_TWO_PI * obj->Ts;

    // a1 = 1/(1+ iqTempVarLpf)
    obj->lpf_a1 = 1.0f / (1.0f + tempVarLpf);

    // b0 = 1 - a1
    obj->lpf_b0 = 1.0f - obj->lpf_a1;

    //
    obj->Kslf = obj->filterFc_sf * MATH_TWO_PI * obj->base_wTs;

    return;
}


//------------------------------------------------------------------------------
void ESMO_updatePLLParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    float32_t speedRefAbs = ti_arm_abs(obj->speedRef);

    obj->pll_Kp = obj->pll_KpMin + obj->pll_KpSF * speedRefAbs;
    obj->pll_Kp = MATH_sat(obj->pll_Kp, obj->pll_KpMax, obj->pll_KpMin);
}

//----------------------------------------------------------------

// end of file
