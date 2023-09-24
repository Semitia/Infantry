#ifndef __CAN1_H
#define __CAN1_H

#include "can.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern CanMsgList_t can1_rx0;
extern CanMsgList_t can1_rx1;

void CAN1_Configuration(void);


#endif




