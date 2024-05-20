//#############################################################################
//
// FILE:  can_msg.h
//
// TITLE: header file for can message
//
//#############################################################################
// $TI Release:
// $Release Date:
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################
#ifndef CAN_MSG_H
#define CAN_MSG_H

#include <drivers/mcan.h>
#include "motor_common.h"


extern MCAN_TxBufElement    MCANtxMsg;
extern MCAN_RxBufElement    MCANrxMsg;

extern MCAN_TxBufElement txMsgs[12];
extern uint32_t txMsgIds[12];
extern MCAN_RxBufElement rxMsg;
extern uint32_t gMcanBaseAddr;

extern volatile uint16_t McanMsgRdy;

extern void HAL_initMcan();
extern void HAL_MCanMsgInit();
extern void HAL_CanMsgInit();
extern void HAL_SendCanMsg(uint16_t msgNum);
extern void HAL_CanRxMsgMapping_0xC0();
extern void HAL_CanTxMsgPacking();
extern void HAL_McanMsgInit(MCAN_TxBufElement *txMsg, uint32_t msgId);

#endif
