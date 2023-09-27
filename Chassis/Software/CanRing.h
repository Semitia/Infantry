#ifndef __CANRING_H__
#define __CANRING_H__

#include "sys.h"   

#define CAN_RING_SIZE 6

typedef struct __CanRing_t {
    CanRxMsg msg[CAN_RING_SIZE];
    short head;
    short tail;
}CanRing_t;

void pushCanMsg(CanRing_t *ring, CanRxMsg new_msg);
uint8_t canRingHasMsg(CanRing_t *ring);
CanRxMsg popCanMsg(CanRing_t *ring);

#endif



