/**
 * @file    CanRing.c
 * @brief   CAN统一接口，环形缓存器实现
 * @version 0.1
 * @date    2023-9-27
 * @author  吴磊
*/

#include"CanRing.h"

// /**
//  * @brief 初始化环形缓存器
//  * @param ring 环形缓存器指针
//  * @retval None
//  * @note 一般不需要调用这个函数，直接CanRing_t ring = {0};即可
//  */
// void canRingInit(CanRing_t *ring) {
//     ring->head = 0;
//     ring->tail = 0;
//     return;
// }

/**
 * @brief 向环形缓存器中添加一个消息
 * @param ring 环形缓存器指针
 * @param new_msg 新消息
 * @retval None
 */
void pushCanMsg(CanRing_t *ring, CanRxMsg new_msg) {
    if((ring->head + 1) % CAN_RING_SIZE == ring->tail) {
        // 缓存器已满，覆盖最旧的消息
        ring->tail = (ring->tail + 1) % CAN_RING_SIZE;
    }
    ring->msg[ring->head] = new_msg;
    ring->head = (ring->head + 1) % CAN_RING_SIZE;
    return;
}

/**
 * @brief 判断环形缓存器中是否有消息
 * @param ring 环形缓存器指针
 * @retval 0:没有消息 1:有消息
 */
uint8_t canRingHasMsg(CanRing_t *ring) {
    if(ring->head == ring->tail) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief 从环形缓存器中取出一个消息
 * @param ring 环形缓存器指针
 * @param msg 消息指针
 * @retval None
 */
CanRxMsg popCanMsg(CanRing_t *ring) {
    CanRxMsg msg = ring->msg[ring->tail];
    ring->tail = (ring->tail + 1) % CAN_RING_SIZE;
    return msg;
}
