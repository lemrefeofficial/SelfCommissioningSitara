//#############################################################################
//
// FILE:   pi.c
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

#include "selfcommm.h"

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(PI_init,"Cla1Prog2");
#endif

////*****************************************************************************
////
//// PI_init
////
////*****************************************************************************
//PI_Handle
//PI1_init(void *pMemory, const size_t numBytes)
//{
//    PI_Handle handle;
//
//    if((int16_t)numBytes < (int16_t)sizeof(PI_Obj))
//    {
//        return((PI_Handle)NULL);
//    }
//
//    //
//    // Assign the handle
//    //
//    handle = (PI_Handle)pMemory;
//
//    return(handle);
//} // end of PI_init() function
//
//// end of file


//*****************************************************************************
//
// SELFCOMM_init
//
//*****************************************************************************
SELFCOMMM_Handle
SELFCOMMM_init(void *pMemory, const size_t numBytes)
{
    SELFCOMMM_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(SELFCOMMM_Obj))
    {
        return((SELFCOMMM_Handle)NULL);
    }

    //
    // Assign the handle
    //
    handle = (SELFCOMMM_Handle)pMemory;

    return(handle);
} // end of PI_init() function

// end of file


// Sets up parameters for angle generation
void SELFCOMM_setParams(SELFCOMMM_Handle handle)
{
    SELFCOMMM_Obj *obj = (SELFCOMMM_Obj *)handle;


    obj->VariableInit = 1.0f;
    obj->InjVolHalfPrdSampleNoMax = 30.0f;
    obj->InjVolHalfPrdSampleNoInit = 8.0f;
    obj->InjSettlePrd = 2.0f;
//    obj->AngIntgGain = 0.0166666f;
    obj->AngIntgGain = 0.0066666f;
    obj->ElecThetaStep = 0.002f;
    obj->InjVolMagnInit = 0.002f;
    obj->IsMagnMinLim = 0.02f;
    obj->IsMagnMaxLim = 0.5f;
    obj->InjVolMagnMaxLim = 0.98f;
    obj->Vdc_Base = 409.0f;
    obj->Vph_Base = 236.1f;
    obj->I_Base = 10.0f;
    obj->F_Base = 100.0f;
    obj->i12 = 0;

    return;
} // end of ANGLE_COMP_setParams() function

// end of the file
