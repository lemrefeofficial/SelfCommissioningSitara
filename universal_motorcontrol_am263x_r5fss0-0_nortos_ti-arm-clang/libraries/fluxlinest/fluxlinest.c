//#############################################################################
//
// FILE:   fluxlinest.c
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

#include <libraries/fluxlinest/fluxlinest.h>

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(PI_init,"Cla1Prog2");
#endif

//****************************************************************************
// FLUXLINEST_init
//*****************************************************************************
FLUXLINEST_Handle FLUXLINEST_init(void *pMemory, const size_t numBytes)
{
    FLUXLINEST_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(FLUXLINEST_Obj))
    {
        return((FLUXLINEST_Handle)NULL);
    }

    //
    // Assign the handle
    //
    handle = (FLUXLINEST_Handle)pMemory;

    return(handle);
} // end of PI_init() function

// end of file

// Sets up parameters for angle generation
void FLUXLINEST_setParams(FLUXLINEST_Handle handle)
{
    FLUXLINEST_Obj *obj = (FLUXLINEST_Obj *)handle;


    obj -> VariableInit = 1.0f;
    obj -> AveragingSize = 20000;
    obj -> WaitingTime2Settle_Speed = 10000.0f;
    obj -> MaxTime4Settle = 100000.0f;
    obj -> TestSpeed = 0.25f;
    obj -> SpeedErrLim4Settle = 0.05f;
    obj -> Rs = 0.11;
    obj -> Lq = 0.21f;
    obj -> TimePU = MATH_TWO_PI *  100.0f * (0.001f/15.0f);
 // obj -> LPF_IqDeriv_Gain = _IQdiv(_IQmpy(_IQ(5.0),Est_MagnFlux_DFT.TimePU), (_IQ(1.0) + _IQmpy(_IQ(5.0), Est_MagnFlux_DFT.TimePU))); // LPF with 500-Hz cut-off - Backward Euler
    obj -> LPF_IqDeriv_Gain = (((5.0f) * obj -> TimePU) / ( 1.0f + (5.0f * obj -> TimePU))); // LPF with 500-Hz cut-off - Backward Euler
 // obj -> LPF_SpeedErr_Gain = _IQdiv(_IQmpy(_IQ(0.01), Est_MagnFlux_DFT.TimePU), (_IQ(1.0) + _IQmpy(_IQ(0.01), Est_MagnFlux_DFT.TimePU))); // LPF with 1-Hz cut-off - Backward Euler
    obj -> LPF_SpeedErr_Gain = ((0.01f * obj -> TimePU) / (1.0f + (0.01f * obj -> TimePU))); // LPF with 1-Hz cut-off - Backward Euler

    obj -> Vdc_Base = 409.0f;
    obj -> Vph_Base = 236.1f;
    obj -> I_Base = 10.0f;
    obj -> F_Base = 100.0f;


    return;
} // end of ANGLE_COMP_setParams() function

// end of the file
