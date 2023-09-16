#ifndef __CAN1_H
#define __CAN1_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 * can消息节点
*/
typedef struct __CanMsgNode_t {
    CanRxMsg msg;
    struct __CanMsgNode_t *next;
    struct __CanMsgNode_t *prev;
} CanMsgNode_t;

/**
 * can消息 双向链表
*/
typedef struct __CanMsgList_t {
    uint8_t num;
    CanMsgNode_t *head;
    CanMsgNode_t *tail;
    SemaphoreHandle_t mutex;    // 互斥信号量
} CanMsgList_t;

extern CanMsgList_t can1_rx0;
extern CanMsgList_t can1_rx1;

void CAN1_Configuration(void);
void addCanMsg(CanMsgList_t* list, CanRxMsg new_msg);
void 

#endif




