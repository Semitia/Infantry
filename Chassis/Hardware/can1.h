#ifndef __CAN1_H
#define __CAN1_H

#include "Can.h"
#include "CanRing.h"

// extern CanMsgList_t can1_rx0;
// extern CanMsgList_t can1_rx1;
extern CanRing_t can1_rx0;
extern CanRing_t can1_rx1;

void CAN1_Configuration(void);

#endif




