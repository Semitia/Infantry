#include "Can.h"

void addCanMsg(CanMsgList_t* list, CanRxMsg new_msg) {
    // 创建一个新的链表节点
    CanMsgNode_t* new_node = (CanMsgNode_t*)pvPortMalloc(sizeof(CanMsgNode_t));
    if (new_node == NULL) {
        // 内存分配失败
        //printf("内存分配失败\r\n");
        xSemaphoreGive(list->mutex);  // 解锁互斥锁
        return;
    }

    // 将新消息存储到新节点中
    new_node->msg = new_msg;
    new_node->next = NULL;

    if (list->head == NULL) {
        // 如果链表为空，新节点就是头节点和尾节点
        new_node->prev = NULL;
        list->head = new_node;
        list->tail = new_node;
    } else {
        // 否则，将新节点添加到尾部
        new_node->prev = list->tail;
        list->tail->next = new_node;
        list->tail = new_node;
    }
    // 更新消息数量
    list->num++;
}

/**
 * @brief 清空链表
 * @param list 链表指针
 * @retval None
 */
void clearList(CanMsgList_t *list) {
    // 获取互斥信号量
    xSemaphoreTake(list->mutex, portMAX_DELAY);

    // 从头开始遍历链表
    CanMsgNode_t *node = list->head;
    while (node != NULL) {
        // 保存下一个节点的指针
        CanMsgNode_t *nextNode = node->next;

        // 释放当前节点的内存
        vPortFree(node);

        // 移动到下一个节点
        node = nextNode;
    }

    // 将链表的头和尾都设置为NULL，表示链表已经为空
    list->head = NULL;
    list->tail = NULL;

    // 释放互斥信号量
    xSemaphoreGive(list->mutex);
}

