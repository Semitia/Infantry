/**********************************************************************************************************
 * @文件     Kinematic.c
 * @说明     常用底盘运动学封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-16
 *********************************************************************************************************/
#include "Kinematic.h"

/****************************************全向轮***********************************************/
#if CHASSIS_TYPE == 0 
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

/**************************************麦克纳母轮***********************************************/
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
	uint8_t i;
    uint8_t received_flag[4] = {0};                          //用于判断某序号的电机是否已经接收到了数据
    CanMsgNode_t *node,*prev_node;
    CanMsgList_t *list = kinematic->can_datalist;              //指向can消息链表的指针
    xSemaphoreTake(list->mutex, portMAX_DELAY);                //获取互斥信号量
    // //开始从后向前遍历
    // node = list->tail;
    // while(node != NULL) {
    //     uint32_t id = node->msg.StdId;
    //     if(!received_flag[id-0x201]) {                      //还没有接收到这个序号的电机数据
    //         received_flag[id-0x201] = 1;
    //         motorUpdate(&(kinematic->motor[id-0x201]), node->msg.Data);
    //     }
    //     prev_node = node->prev;                             //保存上一个节点的指针,释放当前节点的内存
    //     vPortFree(node);
    //     list->num--;
    //     node = prev_node;
    // }
    for(i=0; i<list->num; i++) {
        motorUpdate(&(kinematic->motor[list->msg[list->p].StdId-0x201]), list->msg[list->p].Data);
        list->p--;
        if(list->p>200){
            list->p += CAN_MAX_NUM;
        }
    }
		list->num = 0;
    xSemaphoreGive(list->mutex);                               //释放互斥信号量
}

void updateWheelsFromISR(Kinematic_t *kinematic, CanRxMsg msg) {
    uint8_t id = msg.StdId;
    motorUpdate(&(kinematic->motor[id-0x201]), msg.Data);
}

/**************************************舵轮***********************************************/
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

/**
 * @brief 求两个0-360电机位置的绝对距离，适用于舵轮舵角目标的最短旋转距离
 * @param yaw_now_360 当前位置
 * @param yaw_set_360 目标位置
 * @param reverflag 是否反向
 * @retval 最短移动距离和是否反向，最短旋转距离该函数结果一定能保证不超过90度
*/
float calculateShortestDistance(float yaw_now_360, float yaw_set_360,float* reverflag) {
    float clockwise_distance = fmodf((yaw_set_360 - yaw_now_360 + 360), 360);
    float counter_clockwise_distance = 360 - clockwise_distance;
    float reverse_distance = fabsf(fmodf(yaw_set_360 - yaw_now_360 + 180, 360)) - 180;

    float shortest_distance = clockwise_distance;
	
    if (counter_clockwise_distance < shortest_distance) {
        shortest_distance = -counter_clockwise_distance;
    }
	*reverflag = 1.0;
	//如果反向正向距离都要大于90
	if(ABS(shortest_distance)>90){
		//翻转
		float flipped_yaw_now = yaw_now_360 + 180;
		
		if (flipped_yaw_now >= 360) {
			flipped_yaw_now -= 360;
		}
		if(clockwise_distance > counter_clockwise_distance)//未翻转前正向大于反向，则反转后取反向
		{
			reverse_distance = fmodf((yaw_set_360 - flipped_yaw_now + 360), 360);//求正向距离
		}else{
			reverse_distance = -fmodf(flipped_yaw_now - yaw_set_360 + 360, 360);//求反向距离
		}
		*reverflag = -1.0;
        shortest_distance = reverse_distance;
	}

    return shortest_distance;
}

/**
 * @brief 使用can通信发送舵轮电机角度
*/
void SteerCan1Send(Kinematic_t *kinematic)
{
    CanTxMsg tx_msg;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    tx_msg.StdId = 0x1FF;
    tx.msg.Data[0] = (uint8_t)((kinematic->steering_motor[0].target_current >> 8)&0xff);
    tx.msg.Data[1] = (uint8_t)(kinematic->steering_motor[0].target_current&0xff);
    tx.msg.Data[2] = (uint8_t)((kinematic->steering_motor[1].target_current >> 8)&0xff);
    tx.msg.Data[3] = (uint8_t)(kinematic->steering_motor[1].target_current&0xff);
    tx.msg.Data[4] = (uint8_t)((kinematic->steering_motor[2].target_current >> 8)&0xff);
    tx.msg.Data[5] = (uint8_t)(kinematic->steering_motor[2].target_current&0xff);
    tx.msg.Data[6] = (uint8_t)((kinematic->steering_motor[3].target_current >> 8)&0xff);
    tx.msg.Data[7] = (uint8_t)(kinematic->steering_motor[3].target_current&0xff);
    Can_Transmit(kinematic->can_tx, &tx_msg);
	return;
}

#endif


// /**
//  * @brief 速度向量相加
//  * @param a 向量a
//  * @param b 向量b
//  * @retval 相加后的向量
// */
// Velocity_t addVector(Velocity_t *a, Velocity_t *b) {
//     Velocity_t temp;
//     float a_yaw = DEG2R(a->w);
//     float b_yaw = DEG2R(b->w);
//     float x = a->x * arm_cos_f32(a_yaw) + b->x * arm_cos_f32(b_yaw);
//     float y = a->y * arm_sin_f32(a_yaw) + b->y * arm_sin_f32(b_yaw);
//     arm_sqrt_f32(x * x + y * y,&(temp.x));
//     temp.w = R2DEG(atan2(y, x));
//     return temp;
// }

/**
 * @brief 使用Can通信发送电机期望电流
*/
void setMotorCurrent(Kinematic_t *kinematic) {
    CanTxMsg tx_msg;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    tx_msg.StdId = 0x200;

    tx_msg.Data[0] = (uint8_t)((kinematic->motor[0].target_current >> 8)&0xff);
    tx_msg.Data[1] = (uint8_t)(kinematic->motor[0].target_current&0xff);
    tx_msg.Data[2] = (uint8_t)((kinematic->motor[1].target_current >> 8)&0xff);
    tx_msg.Data[3] = (uint8_t)(kinematic->motor[1].target_current&0xff);
    tx_msg.Data[4] = (uint8_t)((kinematic->motor[2].target_current >> 8)&0xff);
    tx_msg.Data[5] = (uint8_t)(kinematic->motor[2].target_current&0xff);
    tx_msg.Data[6] = (uint8_t)((kinematic->motor[3].target_current >> 8)&0xff);
    tx_msg.Data[7] = (uint8_t)(kinematic->motor[3].target_current&0xff);
    CAN_Transmit(kinematic->can_tx, &tx_msg);
    return;
}
