/**********************************************************************************************************
 * @文件     Kinematic.c
 * @说明     常用底盘运动学封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-16
 **********************************************************************************************************/
#include "Kinematic.h"

#if CHASSIS_TYPE == 0 //全向轮
/**
 * @brief 全向轮底盘正解
 * @param kinematic 运动学结构体
 * @retval None
 */
void forKinematic(Kinematic_t *kinematic) {
    
    return;
}


void invKinematic(Kinematic_t *kinematic) {

    return;
}


#elif CHASSIS_TYPE == 1 //麦克纳母轮

/**
 * @brief 麦克纳母轮底盘初始化
 * @param kinematic 运动学结构体
 * @retval None
*/
void kinematicInit(Kinematic_t *kinematic) {
    int i=0;
    kinematic->can_datalist = &can1_rx0;                    //分配一个can接口
    kinematic->real_vel.x = 0;
    kinematic->real_vel.y = 0;
    kinematic->real_vel.w = 0;
    for(i=0; i<4; i++) {
        motorInit(&(kinematic->motor[i]), RM3508, i);
        kinematic->pid_speed[i].deadband = 0;
        kinematic->pid_speed[i].P        = 20.0f;
        kinematic->pid_speed[i].I        = 1.0f;
        kinematic->pid_speed[i].D        = 0.0f;
        kinematic->pid_speed[i].ErrorMax = 1000.0f;
        kinematic->pid_speed[i].IMax     = 2500.0f;
        kinematic->pid_speed[i].SetPoint = 0.0f;
        kinematic->pid_speed[i].OutMax   = 16000.0f;
    }
    return;
}

void forKinematic(Kinematic_t *kinematic) {
    kinematic->real_vel.x = kinematic->motor[0].speed + kinematic->motor[1].speed + kinematic->motor[2].speed + kinematic->motor[3].speed;
    kinematic->real_vel.y = -kinematic->motor[0].speed + kinematic->motor[1].speed + kinematic->motor[2].speed - kinematic->motor[3].speed;
    kinematic->real_vel.w = (kinematic->motor[0].speed - kinematic->motor[1].speed + kinematic->motor[2].speed - kinematic->motor[3].speed) * 0.25f;
    return;
}

void invKinematic(Kinematic_t *kinematic) {
    kinematic->motor[0].target_speed = + kinematic->target_vel.x + kinematic->target_vel.y - kinematic->target_vel.w;
    kinematic->motor[1].target_speed = - kinematic->target_vel.x + kinematic->target_vel.y - kinematic->target_vel.w;
    kinematic->motor[2].target_speed = + kinematic->target_vel.x - kinematic->target_vel.y - kinematic->target_vel.w;
    kinematic->motor[3].target_speed = - kinematic->target_vel.x - kinematic->target_vel.y - kinematic->target_vel.w;
    return;
}

/**
 * @brief 遍历can消息链表并更新电机数据
 * @param kinematic 运动控制结构体
 * @retval None
 * @note 这里电机报文ID为0x201-0x204，因此可以这样方便地接收。
*/
void updateWheels(Kinematic_t *kinematic) {
    uint8_t received_flag[4] = {0};                          //用于判断某序号的电机是否已经接收到了数据
    CanMsgNode_t *node,*prev_node;
    CanMsgList_t *p = kinematic->can_datalist;              //指向can消息链表的指针
    xSemaphoreTake(p->mutex, portMAX_DELAY);                //获取互斥信号量
    //开始从后向前遍历
    node = p->tail;
    while(node != NULL) {
        uint32_t id = node->msg.StdId;
        if(!received_flag[id-0x201]) {                      //还没有接收到这个序号的电机数据
            received_flag[id-0x201] = 1;
            motorUpdate(&(kinematic->motor[id-0x201]), node->msg.Data);
        }
        prev_node = node->prev;                             //保存上一个节点的指针,释放当前节点的内存
        vPortFree(node);
        node = prev_node;
    }
    xSemaphoreGive(p->mutex);                               //释放互斥信号量
}

#elif CHASSIS_TYPE == 2 //舵轮
void kinematicInit(Kinematic_t *kinematic) {
    int i=0;
    kinematic->can_datalist = &can1_rx0;                    //分配can接口
    kinematic->can_datalist_steering = &can1_rx1;           
    kinematic->real_vel.x = 0;
    kinematic->real_vel.y = 0;
    kinematic->real_vel.w = 0;
    for(i=0; i<4; i++) {
        motorInit(&(kinematic->motor[i]), RM3508, i);
        kinematic->pid_speed[i].deadband = 0;
        kinematic->pid_speed[i].P        = 20.0f;
        kinematic->pid_speed[i].I        = 1.0f;
        kinematic->pid_speed[i].D        = 0.0f;
        kinematic->pid_speed[i].ErrorMax = 1000.0f;
        kinematic->pid_speed[i].IMax     = 2500.0f;
        kinematic->pid_speed[i].SetPoint = 0.0f;
        kinematic->pid_speed[i].OutMax   = 16000.0f;

        motorInit(&(kinematic->steering_motor[i]), RM3508, i+4);
        kinematic->steering_pid_speed[i].deadband = 0;
        kinematic->steering_pid_speed[i].P        = 120.0f;
        kinematic->steering_pid_speed[i].I        = 0.2f;
        kinematic->steering_pid_speed[i].D        = 0.0f;
        kinematic->steering_pid_speed[i].ErrorMax = 8000.0f;
        kinematic->steering_pid_speed[i].IMax     = 16000.0f;
        kinematic->steering_pid_speed[i].SetPoint = 0.0f;
        kinematic->steering_pid_speed[i].OutMax   = 30000.0f;

        kinematic->steering_pid_angle[i].deadband = 2.5f;
        kinematic->steering_pid_angle[i].P        = 7.5f;
        kinematic->steering_pid_angle[i].I        = 0.0f;
        kinematic->steering_pid_angle[i].D        = 0.0f;
        kinematic->steering_pid_angle[i].ErrorMax = 1000.0f;
        kinematic->steering_pid_angle[i].IMax     = 1000.0f;
        kinematic->steering_pid_angle[i].SetPoint = 0.0f;
        kinematic->steering_pid_angle[i].OutMax   = 4000.0f;
        
    }
    return;
}

void forKinematic(Kinematic_t *kinematic) {

    return;
}

void invKinematic(Kinematic_t *kinematic) {

    return;
}

#endif
