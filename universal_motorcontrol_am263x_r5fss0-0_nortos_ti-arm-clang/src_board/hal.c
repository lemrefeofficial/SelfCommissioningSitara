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


//! \file
//! \brief  Contains the various functions related to the HAL object
//!
//

//
// the includes
//
#include "user.h"


//
// drivers
//

// modules

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalog.h"



// **************************************************************************
// the globals
__attribute__ ((section("hal_data"))) HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
__attribute__ ((section("hal_data"))) HAL_Obj       hal;            //!< the hardware abstraction layer object

// **************************************************************************
// the functions



HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    //initialize the ADC handles
    obj->adcHandle[0] = CONFIG_ADC0_BASE_ADDR;
    obj->adcHandle[1] = CONFIG_ADC1_BASE_ADDR;
    obj->adcHandle[2] = CONFIG_ADC2_BASE_ADDR;
    obj->adcHandle[3] = CONFIG_ADC3_BASE_ADDR;
    obj->adcHandle[4] = CONFIG_ADC4_BASE_ADDR;

    // initialize the ADC results
    obj->adcResult[0] = CONFIG_ADC0_RESULT_BASE_ADDR;
    obj->adcResult[1] = CONFIG_ADC1_RESULT_BASE_ADDR;
    obj->adcResult[2] = CONFIG_ADC2_RESULT_BASE_ADDR;
    obj->adcResult[3] = CONFIG_ADC3_RESULT_BASE_ADDR;
    obj->adcResult[4] = CONFIG_ADC4_RESULT_BASE_ADDR;

    // initialize UART handle
    obj->uartHandle = CONFIG_UART1;                      //!< the UART0 handle

    // initialize CAN handle
    obj->canHandle = CONFIG_MCAN0_BASE_ADDR;             //!< the CANA handle

    // initialize DMA handle
    obj->dmaHandle = CONFIG_EDMA0_BASE_ADDR;              //!< the DMA handle

#if defined(DATALOGF4_EN)
    obj->dmaChHandle[0] = DMA_DATALOG1_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_DATALOG2_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[2] = DMA_DATALOG3_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[3] = DMA_DATALOG4_BASE;     //!< the DMA Channel handle
#endif  //DATALOGF4_EN

    // initialize the GPIO toggle
    obj->toggleGPIO[0] = 0;


    // initialize timer handles
    obj->timerHandle[0] = CPU_DIAGNOSTICS_TIMER0_BASE_ADDR;


#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
    // initialize pwmdac handles
    obj->pwmDACHandle[0] = EPWMDAC1_BASE;
    obj->pwmDACHandle[1] = EPWMDAC2_BASE;
    obj->pwmDACHandle[2] = EPWMDAC3_BASE;
    obj->pwmDACHandle[3] = EPWMDAC4_BASE;
    // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE


    return(handle);
} // end of HAL_init() function


HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_MTR_Handle)pMemory;

    // assign the object
    obj = (HAL_MTR_Obj *)handle;

    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
    obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
    obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle

    // initialize CMPSS handle
#if defined(HVMTRPFC_REV1P1)
    obj->cmpssHandle[1] = CONFIG_CMPSS_IV_BASE_ADDR;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CONFIG_CMPSS_IW_BASE_ADDR;    //!< the CMPSS handle
#else   // !(HVMTRPFC_REV1P1)
    obj->cmpssHandle[0] = CONFIG_CMPSS_IU_BASE_ADDR;    //!< the CMPSS handle
    obj->cmpssHandle[1] = CONFIG_CMPSS_IV_BASE_ADDR;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CONFIG_CMPSS_IW_BASE_ADDR;    //!< the CMPSS handle
#endif  // !(HVMTRPFC_REV1P1)

#if defined(MOTOR1_HALL)
    // initialize CAP handles for Motor 1
    obj->capHandle[0] = MTR1_CAP_U_BASE;        //!< the CAP handle
    obj->capHandle[1] = MTR1_CAP_V_BASE;        //!< the CAP handle
    obj->capHandle[2] = MTR1_CAP_W_BASE;        //!< the CAP handle
#endif // MOTOR1_HALL


    // Assign gateEnableGPIO
#if defined(DRV8329AEVM_REVA)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;
    obj->gateSleepGPIO = MTR1_GATE_nSLEEP_GPIO;
    obj->gateSleepGPIOBaseAdd = MTR1_GATE_nSLEEP_GPIO_BASE_ADD;

#elif defined(BSXL8323RS_REVA)
    // initialize drv8323 interface
    obj->drvicHandle = DRVIC_init(&obj->drvic);

    // initialize SPI handle
    obj->spiHandle = MTR1_SPI_BASE;             //!< the SPI handle

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;
    obj->gateCalGPIO = MTR1_GATE_CAL_GPIO;
    obj->gateCalGPIOBaseAdd = MTR1_GATE_CAL_GPIO_BASE_ADD;

#elif defined(BSXL8323RH_REVB)
    obj->gateModeGPIO = MTR1_GATE_MODE_GPIO;
    obj->gateModeGPIOBaseAdd = MTR1_GATE_MODE_GPIO_BASE_ADD;
    obj->gateGainGPIO = MTR1_GATE_GAIN_GPIO;
    obj->gateGainGPIOBaseAdd = MTR1_GATE_GAIN_GPIO_BASE_ADD;
    obj->gateCalGPIO = MTR1_GATE_CAL_GPIO;
    obj->gateCalGPIOBaseAdd = MTR1_GATE_CAL_GPIO_BASE_ADD;
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;

#elif defined(BSXL8353RS_REVA)
    // initialize drv8353interface
    obj->drvicHandle = DRVIC_init(&obj->drvic);

    // initialize SPI handle
    obj->spiHandle = MTR1_SPI_BASE;             //!< the SPI handle

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;

#elif defined(BSXL8316RT_REVA)
    // initialize drv8316 interface
    obj->drvicHandle = DRVIC_init(&obj->drvic);

    // initialize SPI handle
    obj->spiHandle = MTR1_SPI_BASE;             //!< the SPI handle

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;
    obj->gateSleepGPIO = MTR1_DRV_SLEEP_GPIO;
    obj->gateSleepGPIOBaseAdd = MTR1_DRV_SLEEP_GPIO_BASE_ADD;
    // enable the DRV device

    GPIO_pinWriteLow(obj->gateSleepGPIOBaseAdd, obj->gateSleepGPIO);   // 1-Active,  0-Low Power Sleep Mode

    ClockP_usleep(10L);                 // delay 10us

    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);   // 1-Disable, 0-Enable

    ClockP_usleep(10L);                 // delay 10us

    GPIO_pinWriteHigh(obj->gateSleepGPIOBaseAdd, obj->gateSleepGPIO);   // 1-Active,  0-Low Power Sleep Mode

    // BSXL3PHGAN_REVA
#elif defined(BSXL3PHGAN_REVA)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;

    // HVMTRPFC_REV1P1
#elif defined(HVMTRPFC_REV1P1)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;
#endif  // Assign gateEnableGPIO

    // initialize QEP driver
    obj->qepHandle = MTR1_QEP_BASE;             // the QEP handle


    obj->motorNum = MTR_1;

    return(handle);
} // end of HAL_MTR1_init() function







void HAL_setParams(HAL_Handle handle)
{

#if defined(EPWMDAC_MODE)
    // setup the PWM DACs
    HAL_setupPWMDACs(handle, USER_SYSTEM_FREQ_MHz);
#endif  //EPWMDAC_MODE

    return;
} // end of HAL_setParams() function




void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams)
{
    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);
    HAL_setNumVoltageSensors(handle, pUserParams->numVoltageSensors);

    // setup the PWMs
    HAL_setupPWMs(handle);


#if defined(MOTOR1_ENC)
    // setup the EQEP
    HAL_setupQEP(handle);
#endif  // MOTOR1_ENC

    // setup faults
    HAL_setupMtrFaults(handle);

#if defined(MOTOR1_HALL)
    // setup the CAPs
    HAL_setupCAPs(handle);
#endif  // MOTOR1_HALL

#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)

    // setup the spi for drv8323/drv8353/drv8316
    HAL_setupSPI(handle);
#endif  // BSXL8323RS_REVA || BSXL8353RS_REVA || BSXL8316RT_REVA

    // disable the PWM
    HAL_disablePWM(handle);

    return;
} // end of HAL_MTR_setParams() function






#if defined(MOTOR1_HALL)
void HAL_setupCAPs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // Disable ,clear all capture flags and interrupts
        ECAP_disableInterrupt(obj->capHandle[cnt],
                              (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                               ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                               ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                               ECAP_ISR_SOURCE_COUNTER_COMPARE));

        ECAP_clearInterrupt(obj->capHandle[cnt],
                            (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                             ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                             ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                             ECAP_ISR_SOURCE_COUNTER_COMPARE));

        // Disable CAP1-CAP4 register loads
        ECAP_disableTimeStampCapture(obj->capHandle[cnt]);

        // Configure eCAP
        //    Enable capture mode.
        //    One shot mode, stop capture at event 3.
        //    Set polarity of the events to rising, falling, rising edge.
        //    Set capture in time difference mode.
        //    Select input from XBAR4/5/6.
        //    Enable eCAP module.
        //    Enable interrupt.
        ECAP_stopCounter(obj->capHandle[cnt]);
        ECAP_enableCaptureMode(obj->capHandle[cnt]);

        // Sets eCAP in Capture mode
        ECAP_setCaptureMode(obj->capHandle[cnt], ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_3);

        // Sets the Capture event prescaler.
        ECAP_setEventPrescaler(obj->capHandle[cnt], 0U);

        // Sets a phase shift value count.
        ECAP_setPhaseShiftCount(obj->capHandle[cnt], 0U);

        // Sets the Capture event polarity
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_3, ECAP_EVNT_FALLING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

        // Configure counter reset on events
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_1);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_2);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_3);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_4);

        // Configures emulation mode.
        ECAP_setEmulationMode(obj->capHandle[cnt], ECAP_EMULATION_FREE_RUN);

        // Enable counter loading with phase shift value.
        ECAP_enableLoadCounter(obj->capHandle[cnt]);

        // Configures Sync out signal mode.
        ECAP_setSyncOutMode(obj->capHandle[cnt], ECAP_SYNC_OUT_SYNCI);

        // Set up the source for sync-in pulse
        ECAP_setSyncInPulseSource(obj->capHandle[cnt], ECAP_SYNC_IN_PULSE_SRC_DISABLE);

        // Starts Time stamp counter
        ECAP_startCounter(obj->capHandle[cnt]);

        // Enables time stamp capture
        ECAP_enableTimeStampCapture(obj->capHandle[cnt]);

        // Re-arms the eCAP module
        ECAP_reArm(obj->capHandle[cnt]);
    }

    return;
}   // HAL_setupCAPs()



void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        ECAP_setAPWMPeriod(obj->capHandle[cnt], 0);
        ECAP_setAPWMCompare(obj->capHandle[cnt], 0x01FFFFFF);
        ECAP_setAPWMShadowPeriod(obj->capHandle[cnt], 0x01FFFFFF);
        ECAP_setAPWMShadowCompare(obj->capHandle[cnt], 0x01FFFFFF);
    }

    return;
}   // HAL_resetCAPTimeStamp()

#endif  // MOTOR1_HALL


// HAL_setupGate & HAL_enableDRV
#if defined(DRV8329AEVM_REVA)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set nSLEEP to low for clear the fault
    GPIO_pinWriteLow(obj->gateSleepGPIOBaseAdd, obj->gateSleepGPIO);

    ClockP_usleep(2L);      // delay 2us

    // Set EN_GATE to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    ClockP_usleep(2L);      // 2.0us

    // Set nSLEEP to high for wake the device
    GPIO_pinWriteHigh(obj->gateSleepGPIOBaseAdd, obj->gateSleepGPIO);

    ClockP_usleep(2L);      // delay 2us

    // Set EN_GATE to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    return;
} // HAL_setupGate() function

#elif defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    DRVIC_enable(obj->drvicHandle);

    ClockP_usleep(1000L);      // delay 1000us

    return;
}  // end of HAL_enableDRV() function

void HAL_writeDRVData(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars)
{
    HAL_MTR_Obj  *obj = (HAL_MTR_Obj *)handle;

    DRVIC_writeData(obj->drvicHandle, drvicVars);

    return;
}  // end of HAL_writeDRVData() function

void HAL_readDRVData(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars)
{
    HAL_MTR_Obj  *obj = (HAL_MTR_Obj *)handle;

    DRVIC_readData(obj->drvicHandle, drvicVars);

    return;
}  // end of HAL_readDRVData() function


void HAL_setupDRVSPI(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars)
{
    HAL_MTR_Obj  *obj = (HAL_MTR_Obj *)handle;

    DRVIC_setupSPI(obj->drvicHandle, drvicVars);

    return;
}  // end of HAL_setupDRVSPI() function

void HAL_setupGate(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    DRVIC_setSPIHandle(obj->drvicHandle, obj->spiHandle);

    DRVIC_setGPIOCSNumber(obj->drvicHandle, MTR1_DRV_SPI_CS_GPIO, MTR1_DRV_SPI_CS_GPIO_BASE_ADD);
    DRVIC_setGPIOENNumber(obj->drvicHandle, MTR1_GATE_EN_GPIO, MTR1_GATE_CAL_GPIO_BASE_ADD);

    return;
} // HAL_setupGate() function





void HAL_setupSPI(HAL_MTR_Handle handle)
{
//    HAL_MTR_Obj   *obj = (HAL_MTR_Obj *)handle;
//
//    // Must put SPI into reset before configuring it
//    SPI_disableModule(obj->spiHandle);
//
//    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size, 30MHz LSPCLK
//    SPI_setConfig(obj->spiHandle, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
//                  SPI_MODE_MASTER, 400000, 16);
//
//    SPI_disableLoopback(obj->spiHandle);
//
//    SPI_setEmulationMode(obj->spiHandle, SPI_EMULATION_FREE_RUN);
//
//    SPI_enableFIFO(obj->spiHandle);
//    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX0, SPI_FIFO_RX0);
//    SPI_setTxFifoTransmitDelay(obj->spiHandle, 0x10);
//
//    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);
//
//    // Configuration complete. Enable the module.
//    SPI_enableModule(obj->spiHandle);

    return;
}  // end of HAL_setupSPI() function






// BSXL8323RH_REVB
#elif defined(BSXL8323RH_REVB)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to high for enabling the DRV
    GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);


    // Set MODE to low for setting 6-PWM mode
    GPIO_pinWriteLow(obj->gateModeGPIOBaseAdd, obj->gateModeGPIO);


    // disable calibrate mode
    GPIO_pinWriteLow(obj->gateCalGPIOBaseAdd, obj->gateCalGPIO);


    return;
} // HAL_setupGate() function




// BSXL3PHGAN_REVA
#elif defined(BSXL3PHGAN_REVA)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE (nEN_uC) to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    return;
} // HAL_setupGate() function







// HVMTRPFC_REV1P1
#elif defined(HVMTRPFC_REV1P1)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    return;
} // HAL_setupGate() function
// HVMTRPFC_REV1P1
#else
#error No HAL_setupGate or HAL_enableDRV
#endif  // HAL_setupGate & HAL_enableDRV








void HAL_setupMtrFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t cnt;

    for(cnt=0; cnt<3; cnt++)
    {
        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], HAL_TZFLAG_INTERRUPT_ALL);
    }


    // Clear any spurious fault
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_setupMtrFaults() function




void HAL_setupGPIOs(HAL_Handle handle)
{


#if defined(BSXL3PHGAN_REVA)

//    GPIO_pinWriteLow(CONFIG_DRV_CAL_BASE_ADDR, CONFIG_DRV_CAL_PIN);
    GPIO_pinWriteLow(CONFIG_LED1C_BASE_ADDR, CONFIG_LED1C_PIN);
    GPIO_pinWriteLow(CONFIG_LED1B_BASE_ADDR, CONFIG_LED1B_PIN);
    GPIO_pinWriteLow(CONFIG_LED2B_BASE_ADDR, CONFIG_LED2B_PIN);

#endif //defined(BSXL3PHGAN_REVA)


    return;
}  // end of HAL_setupGPIOs() function




void HAL_setupPWMs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    uint16_t pwmPeriodCycles = (uint16_t)(USER_M1_PWM_TBPRD_NUM);
    uint16_t numPWMTicksPerISRTick = USER_M1_NUM_PWM_TICKS_PER_ISR_TICK;

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA) || \
    defined(BSXL3PHGAN_REVA) || defined(HVMTRPFC_REV1P1) || \
    defined(DRV8329AEVM_REVA) || defined(AM2BLDCSERVO)

    for(cnt=0; cnt<3; cnt++)
    {


#if defined(MOTOR1_DCLINKSS)
        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
#else  // !(MOTOR1_DCLINKSS)


#endif  // !(MOTOR1_DCLINKSS)

    }

    // BSXL8323RS_REVA || BSXL8323RH_REVB || BSXL8353RS_REVA || \
    // BSXL8316RT_REVA || BSXL3PHGAN_REVA || HVMTRPFC_REV1P1 || \
    // DRV8329AEVM_REVA
#else
#error The PWM is not configured for motor_1 control
#endif  // boards

#if defined(MOTOR1_DCLINKSS)
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_ZERO);

    EPWM_enableInterrupt(obj->pwmHandle[0]);

    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    // ADC SOC trigger for the 1st dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_U_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_A);

    // ADC SOC trigger for the 2nd dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                             EPWM_SOC_B,
                             EPWM_SOC_TBCTR_U_CMPD);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_B);

    // ADC SOC trigger for the 3rd dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[2],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_A);

    // ADC SOC trigger for the 4th dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[2],
                             EPWM_SOC_B,
                             EPWM_SOC_TBCTR_D_CMPD);

    EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_B);
#else   //!(MOTOR1_DCLINKSS)
    // setup the Event Trigger Selection Register (ETSEL)




#endif  // !(MOTOR1_DCLINKSS)



    // setup the Event Trigger Prescale Register (ETPS)
    if(numPWMTicksPerISRTick > 15)
    {
        numPWMTicksPerISRTick = 15;
    }
    else if(numPWMTicksPerISRTick < 1)
    {
        numPWMTicksPerISRTick = 1;
    }

    EPWM_setInterruptEventCount(obj->pwmHandle[0], numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                    numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                    numPWMTicksPerISRTick);

#if defined(MOTOR1_DCLINKSS)
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A,
                                    numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B,
                                    numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A,
                                    numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B,
                                    numPWMTicksPerISRTick);
#endif  //MOTOR1_DCLINKSS

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_B);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], pwmPeriodCycles);

    // write the PWM data value  for ADC trigger
    // EPWM1 is pinouted for phase B
    EPWM_setCounterCompareValue(obj->pwmHandle[1], EPWM_COUNTER_COMPARE_C, 10);

    // sync "down-stream"
//    EPWM_enableSyncOutPulseSource(obj->pwmHandle[0], EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    // write the PWM data value  for ADC trigger
#if defined(MOTOR1_DCLINKSS)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_B);

    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_B);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, pwmPeriodCycles>>1);
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, pwmPeriodCycles>>1);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, pwmPeriodCycles>>1);
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, pwmPeriodCycles>>1);

#endif  //MOTOR1_DCLINKSS


    return;
}  // end of HAL_setupPWMs() function




#if defined(EPWMDAC_MODE)
void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;

    // PWMDAC frequency = 100kHz, calculate the period for pwm:2000
    uint16_t  period_cycles = (uint16_t)(systemFreq_MHz *
                                  (float32_t)(1000.0f / HA_PWMDAC_FREQ_KHZ));
    uint16_t  cnt;

    for(cnt = 0; cnt < 4; cnt++)
    {


        EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], period_cycles);

    }

    return;
}  // end of HAL_setupPWMDACs() function
#endif  // EPWMDAC_MODE









#if defined(MOTOR1_ENC)
void HAL_setupQEP(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj   *obj = (HAL_MTR_Obj *)handle;

//    // Configure the decoder for quadrature count mode, counting both
//    // rising and falling edges (that is, 2x resolution), QDECCTL
//    EQEP_setDecoderConfig(obj->qepHandle, (EQEP_CONFIG_2X_RESOLUTION |
//                                           EQEP_CONFIG_QUADRATURE |
//                                           EQEP_CONFIG_NO_SWAP) );
//
//    EQEP_setEmulationMode(obj->qepHandle, EQEP_EMULATIONMODE_RUNFREE);
//
//    // Configure the position counter to be latched on a unit time out
//    // and latch on rising edge of index pulse
//    EQEP_setLatchMode(obj->qepHandle, (EQEP_LATCH_RISING_INDEX |
//                                       EQEP_LATCH_UNIT_TIME_OUT) );

    //update the maximum posistion for reset based on user motor value
    EQEP_setPositionCounterConfig(obj->qepHandle, EQEP_POSITION_RESET_MAX_POS,
                                  (uint32_t)(USER_MOTOR1_ENC_POS_MAX));

#if defined(ENC_UVW)
    EQEP_setInitialPosition(obj->qepHandle, USER_MOTOR1_ENC_POS_OFFSET);

    EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_RISING_INDEX);
#endif

    // Enable the unit timer, setting the frequency to 10KHz
    // QUPRD, QEPCTL
    EQEP_enableUnitTimer(obj->qepHandle, (USER_M1_QEP_UNIT_TIMER_TICKS - 1));

//    // Disables the eQEP module position-compare unit
//    EQEP_disableCompare(obj->qepHandle);
//
//    // Configure and enable the edge-capture unit. The capture clock divider is
//    // SYSCLKOUT/128. The unit-position event divider is QCLK/32.
//    EQEP_setCaptureConfig(obj->qepHandle, EQEP_CAPTURE_CLK_DIV_128,
//                                          EQEP_UNIT_POS_EVNT_DIV_32);
//
//    // Enable QEP edge-capture unit
//    EQEP_enableCapture(obj->qepHandle);
//
//    // Enable UTO on QEP
//    EQEP_enableInterrupt(obj->qepHandle, EQEP_INT_UNIT_TIME_OUT);

    // Enable the eQEP module
    EQEP_enableModule(obj->qepHandle);

    return;
}
#endif  // MOTOR1_ENC





//commented in the beginning of this file
void HAL_setupSCIA(HAL_Handle halHandle)
{
//    HAL_Obj *obj = (HAL_Obj *)halHandle;
//
//    // Initialize SCIA and its FIFO.
//    SCI_performSoftwareReset(obj->sciHandle);
//
//    // Configure SCIA for echoback.
//    SCI_setConfig(obj->sciHandle, DEVICE_LSPCLK_FREQ, 9600,
//                        ( SCI_CONFIG_WLEN_8 |
//                          SCI_CONFIG_STOP_ONE |
//                          SCI_CONFIG_PAR_NONE ) );
//
//    SCI_resetChannels(obj->sciHandle);
//
//    SCI_resetRxFIFO(obj->sciHandle);
//
//    SCI_resetTxFIFO(obj->sciHandle);
//
//    SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_TXFF | SCI_INT_RXFF);
//
//    SCI_enableFIFO(obj->sciHandle);
//
//    SCI_enableModule(obj->sciHandle);
//
//    SCI_performSoftwareReset(obj->sciHandle);

    return;
}  // end of DRV_setupSci() function



void HAL_setupI2CA(HAL_Handle halHandle)
{
//    HAL_Obj *obj = (HAL_Obj *)halHandle;
//
//    // Must put I2C into reset before configuring it
//    I2C_disableModule(obj->i2cHandle);
//
//    // I2C configuration. Use a 400kHz I2CCLK with a 50% duty cycle.
//    I2C_initMaster(obj->i2cHandle, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_50);
//    I2C_setConfig(obj->i2cHandle, I2C_MASTER_SEND_MODE);
//    I2C_setSlaveAddress(obj->i2cHandle, I2C_SLAVE_ADDRESS);
//    I2C_disableLoopback(obj->i2cHandle);
//    I2C_setBitCount(obj->i2cHandle, I2C_BITCOUNT_8);
//    I2C_setDataCount(obj->i2cHandle, 2);
//    I2C_setAddressMode(obj->i2cHandle, I2C_ADDR_MODE_7BITS);
//
//    // Enable stop condition and register-access-ready interrupts
//    I2C_enableInterrupt(obj->i2cHandle, I2C_INT_ADDR_SLAVE |
//                                        I2C_INT_ARB_LOST |
//                                        I2C_INT_NO_ACK |
//                                        I2C_INT_STOP_CONDITION);
//
//    // FIFO configuration
//    I2C_enableFIFO(obj->i2cHandle);
//    I2C_setFIFOInterruptLevel(obj->i2cHandle, I2C_FIFO_TXEMPTY, I2C_FIFO_RX2);
//
////    I2C_clearInterruptStatus(obj->i2cHandle, I2C_INT_RXFF | I2C_INT_TXFF);
//    I2C_clearInterruptStatus(obj->i2cHandle, I2C_INT_ARB_LOST | I2C_INT_NO_ACK);
//
//    // Configuration complete. Enable the module.
//    I2C_setEmulationMode(obj->i2cHandle, I2C_EMULATION_FREE_RUN);
//    I2C_enableModule(obj->i2cHandle);

    return;
}  // end of HAL_setupI2CA() function



void HAL_setupTimeBaseTimer(HAL_Handle handle, const float32_t timeBaseFreq_Hz)
{
//    HAL_Obj  *obj = (HAL_Obj *)handle;
//
//    uint32_t timerPeriod = (uint32_t)((USER_SYSTEM_FREQ_MHz * 1000.0f *1000.0f) /
//                                      timeBaseFreq_Hz);
//
//    // use timer 0 for CPU usage diagnostics
//    CPUTimer_setPreScaler(obj->timerHandle[0], 0);
//
//    CPUTimer_setEmulationMode(obj->timerHandle[0],
//                              CPUTIMER_EMULATIONMODE_RUNFREE);
//
//    CPUTimer_setPeriod(obj->timerHandle[0], timerPeriod);
//
//    CPUTimer_startTimer(obj->timerHandle[0]);

    return;
}  // end of HAL_setupTimeBaseTimer() function



void HAL_setupADCTriggerTimer(HAL_Handle handle, const float32_t adcTriggerFreq_Hz)
{
//    HAL_Obj  *obj = (HAL_Obj *)handle;
//
//    uint32_t timerPeriod = (uint32_t)((USER_SYSTEM_FREQ_MHz * 1000.0f *1000.0f) /
//                                      adcTriggerFreq_Hz);
//
//    // use timer 1 for ADC trigger
//    CPUTimer_setPreScaler(obj->timerHandle[1], 0);
//
//    CPUTimer_setEmulationMode(obj->timerHandle[1],
//                              CPUTIMER_EMULATIONMODE_RUNFREE);
//
//    CPUTimer_setPeriod(obj->timerHandle[1], timerPeriod);
//
//    CPUTimer_enableInterrupt(obj->timerHandle[1]);
//
//    CPUTimer_startTimer(obj->timerHandle[1]);

    return;
}  // end of HAL_setupADCTriggerTimer() function


//




void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_DCLINKSS)
#if defined(DRV8329AEVM_REVA)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    // DRV8329AEVM_REVA
#else
#error This board doesn't support single shunt
#endif  // BSXL8323RS_REVA || BSXL8323RH_REVB || DRV8329AEVM_REVA
#else   // !(MOTOR1_DCLINKSS)
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB) || \
    defined(BSXL8353RS_REVA)

    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
    // BSXL8323RS_REVA | BSXL8323RH_REVB | BSXL8353RS_REVA
#elif defined(HVMTRPFC_REV1P1) || defined(BSXL8316RT_REVA) || \
      defined(BSXL3PHGAN_REVA)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[2], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
    // HVMTRPFC_REV1P1 | BSXL8316RT_REVA | BSXL3PHGAN_REVA
#else
#error Not Select a Right Board
#endif  // 3-Shunt Hardware Board
#endif  // !(MOTOR1_DCLINKSS)

    return;
}   // end of HAL_setMtrCMPSSDACValue() function





void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us,
                   const float32_t adcSample_us)
{
    uint16_t deadband =  (uint16_t)(deadband_us * systemFreq_MHz);
    uint16_t noiseWindow =  (uint16_t)(noiseWindow_us * systemFreq_MHz);
    uint16_t adcSample =  (uint16_t)(adcSample_us * systemFreq_MHz);

    pPWMData->deadband = deadband;
    pPWMData->noiseWindow = noiseWindow;
    pPWMData->adcSample = adcSample;

    pPWMData->minCMPValue = deadband + noiseWindow + adcSample;

    return;
}   // end of HAL_setTriggerPrams() function




Bool HAL_MTR_setGateDriver(HAL_MTR_Handle handle)
{
    Bool driverStatus = FALSE;

    ClockP_usleep(5000L);

    // Setup Gate Enable
#if defined(DRV8329AEVM_REVA)
    // turn on the DRV8329A if present
    HAL_enableDRV(handle);

    // DRV8329AEVM_REVA
#elif defined(BSXL8323RS_REVA)
    // enable DRV8323RS
    HAL_setupGate(handle);
    ClockP_usleep(1000U);

    // turn on the DRV8323RS
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    // initialize the DRV8323RS interface
    HAL_setupDRVSPI(handle, &drvicVars_M1);

    ClockP_usleep(1000U);

    drvicVars_M1.ctrlReg02.bit.OTW_REP = TRUE;
    drvicVars_M1.ctrlReg02.bit.PWM_MODE = DRV8323_PWMMODE_6;

    drvicVars_M1.ctrlReg05.bit.VDS_LVL = DRV8323_VDS_LEVEL_1P700_V;
    drvicVars_M1.ctrlReg05.bit.OCP_MODE = DRV8323_AUTOMATIC_RETRY;
    drvicVars_M1.ctrlReg05.bit.DEAD_TIME = DRV8323_DEADTIME_100_NS;

    drvicVars_M1.ctrlReg06.bit.CSA_GAIN = DRV8323_Gain_10VpV;         // ADC_FULL_SCALE_CURRENT = 47.14285714A
//    drvicVars_M1.ctrlReg06.bit.CSA_GAIN = DRV8323_Gain_20VpV;       // ADC_FULL_SCALE_CURRENT = 23.57142857A
    drvicVars_M1.ctrlReg06.bit.LS_REF = FALSE;
    drvicVars_M1.ctrlReg06.bit.VREF_DIV = TRUE;
    drvicVars_M1.ctrlReg06.bit.CSA_FET = FALSE;

    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    // BSXL8323RS_REVA
#elif defined(BSXL8323RH_REVB)
    // turn on the DRV8323RH if present
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    // BSXL8323RH_REVB
#elif defined(BSXL8353RS_REVA)
    // enable DRV8353RS
    HAL_setupGate(handle);
    ClockP_usleep(1000U);

    // turn on the DRV8353RS
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    // initialize the DRV8353RS interface
    HAL_setupDRVSPI(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    drvicVars_M1.ctrlReg03.bit.IDRIVEP_HS = DRV8353_ISOUR_HS_0P820_A;
    drvicVars_M1.ctrlReg03.bit.IDRIVEN_HS = DRV8353_ISINK_HS_1P640_A;

    drvicVars_M1.ctrlReg04.bit.IDRIVEP_LS = DRV8353_ISOUR_LS_0P820_A;
    drvicVars_M1.ctrlReg04.bit.IDRIVEN_LS = DRV8353_ISINK_LS_1P640_A;

    drvicVars_M1.ctrlReg05.bit.VDS_LVL = DRV8353_VDS_LEVEL_1P500_V;
    drvicVars_M1.ctrlReg05.bit.OCP_MODE = DRV8353_LATCHED_SHUTDOWN;
    drvicVars_M1.ctrlReg05.bit.DEAD_TIME = DRV8353_DEADTIME_100_NS;

    drvicVars_M1.ctrlReg06.bit.CSA_GAIN = DRV8353_Gain_10VpV;
    drvicVars_M1.ctrlReg06.bit.LS_REF = FALSE;
    drvicVars_M1.ctrlReg06.bit.VREF_DIV = TRUE;
    drvicVars_M1.ctrlReg06.bit.CSA_FET = FALSE;

    // write DRV8353RS control registers
    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    // write DRV8353RS control registers again
    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    // BSXL8353RS_REVA
#elif defined(BSXL8316RT_REVA)
    // enable DRV8316RT
    HAL_setupGate(handle);
    ClockP_usleep(1000U);

    // turn on the DRV8316RT
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    // initialize the DRV8316RT interface
    HAL_setupDRVSPI(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    drvicVars_M1.ctrlReg02.bit.PWM_MODE = DRV8316_PWMMODE_6_N;
    drvicVars_M1.ctrlReg02.bit.SLEW = DRV8316_SLEW_50V;

    // Don't change the CSA_GAIN! If changes this setting value,
    // USER_M1_ADC_FULL_SCALE_CURRENT_A must be changed in user_mtr1.h accordingly
    drvicVars_M1.ctrlReg05.bit.CSA_GAIN = DRV8316_CSA_GAIN_0p15VpA;

    drvicVars_M1.ctrlReg06.bit.BUCK_DIS = TRUE;
    drvicVars_M1.ctrlReg06.bit.BUCK_SEL = DRV8316_BUCK_SEL_3p3V;

    // write DRV8316RT control registers
    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    // write DRV8316RT control registers again
    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(handle, &drvicVars_M1);
    ClockP_usleep(1000U);

    // BSXL8316RT_REVA
#elif defined(BSXL3PHGAN_REVA)
    // turn on the 3PhGaN if present
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    // BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
    // turn on the HvKit if present
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    //HVMTRPFC_REV1P1
#else
#error Not select a right supporting kit!
#endif  // Setup Gate Enable

    return(driverStatus);
}

// end of file
