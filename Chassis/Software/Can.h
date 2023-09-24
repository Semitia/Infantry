#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

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

void addCanMsg(CanMsgList_t* list, CanRxMsg new_msg);
void clearList(CanMsgList_t *list);
#endif

