#include "Chassis.h"

/**
 * @brief  底盘初始化
 * @param  Chassis_t *chassis 底盘结构体指针
 * @param  ChassisTypeEnum type_enum 底盘类型枚举
 * @retval None
*/
void chassisInit(Chassis_t *chassis, ChassisTypeEnum type_enum) {
    chassis->type_enum = type_enum;
    chassis->gimbal_canRx = &can2_rx0;
    kinematicInit(&(chassis->kinematic));

    
    return;
}

/**
 * @brief  通过分配的can数据链表读取云台数据
 * @param  Chassis_t *chassis 底盘结构体指针
 * @retval None
 * @note   数据消息ID为0x101~0x102
*/
void recvGimData(Chassis_t *chassis) {
    uint8_t recv_flag[2] = {0};
    CanMsgNode_t *node, *prev_node;
    CanMsgList_t *p = chassis->gimbal_canRx;
    xSemaphoreTake(p->mutex, portMAX_DELAY);
    //从后向前遍历（因为新消息被添加到尾部）
    node = p->tail;
    while(node != NULL) {
        uint32_t id = node->msg.StdId;
        if(!recv_flag[id-0x101]) {
            recv_flag[id-0x101] = 1;
            switch(id) {
                case 0x101:
                    chassis->kinematic.target_vel.x = (node->msg.Data[0]<<8 | node->msg.Data[1]);
                    chassis->kinematic.target_vel.y = (node->msg.Data[2]<<8 | node->msg.Data[3]);
                    chassis->kinematic.target_vel.w = (node->msg.Data[4]<<8 | node->msg.Data[5]);
                    chassis->yaw_100 = (node->msg.Data[6]<<8 | node->msg.Data[7]);
                    break;
                case 0x102:
                    chassis->state.sup_power_flag = node->msg.Data[0];
                    chassis->state.chassis_mode = node->msg.Data[1];
                    chassis->pitch_100 = (node->msg.Data[2]<<8 | node->msg.Data[3]);
                    chassis->state.gimbal_mode = node->msg.Data[4];
                    chassis->state.auto_fire = (node->msg.Data[5]>>0)&0x01;
                    chassis->state.laser_flag = (node->msg.Data[5]>>1)&0x01;
                    chassis->state.graphic_init_flag = (node->msg.Data[5]>>2)&0x01;
                    chassis->state.fire_freq = (node->msg.Data[5]>>3)&0x01;
                    chassis->state.fric_flag = (node->msg.Data[5]>>4)&0x01;
                    chassis->state.enemy_id = (node->msg.Data[5]>>5)&0x07;
                    break;
                default:
                    break;
            }
        }
        prev_node = node->prev;
        vPortFree(node);
        node = prev_node;
    }
    xSemaphoreGive(p->mutex);
    return;
}

