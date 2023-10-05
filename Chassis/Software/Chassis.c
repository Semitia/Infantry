#include "Chassis.h"

/**
 * @brief  底盘初始化
 * @param  Chassis_t *chassis 底盘结构体指针
 * @param  ChassisTypeEnum type_enum 底盘类型枚举
 * @retval None
*/
void chassisInit(Chassis_t *chassis, ChassisTypeEnum type_enum) {
    chassis->type_enum = type_enum;
    chassis->gimbal_canRx = GIMBAL_CAN_RX;
    chassis->gimbal_canTx = GIMBAL_CAN_TX;
    judgeInit(&(chassis->judge), JUDGE_USART_IF);
    kinematicInit(&(chassis->kinematic), KINE_CAN_TX, KINE_CAN_RX);
    return;
}

/**
 * @brief  通过分配的can数据缓存器读取云台数据
 * @param  Chassis_t *chassis 底盘结构体指针
 * @retval None
 * @note   数据消息ID为0x101~0x102
*/
void recvGimData(Chassis_t *chassis) {          //环形缓存器版本
    short raw[3];                               //用于存储原始数据
    uint8_t recv_flag[2] = {0};
    CanRxMsg msg[2];                            //用于存储最新can消息
    CanRing_t *p = chassis->gimbal_canRx;       //指向can消息环形缓存器的指针
    while(canRingHasMsg(p)) {
        CanRxMsg temp_msg = popCanMsg(p);
        msg[temp_msg.StdId-0x101] = temp_msg;
        recv_flag[temp_msg.StdId-0x101] = 1;
    }
    if(recv_flag[0]) {                          //0x101
        raw[0] = (short)(msg[0].Data[0]<<8 | msg[0].Data[1]);
        raw[1] = (short)(msg[0].Data[2]<<8 | msg[0].Data[3]);
        raw[2] = (short)(msg[0].Data[4]<<8 | msg[0].Data[5]);
        chassis->kinematic.target_vel.x = raw[0]/1000.0f;
        chassis->kinematic.target_vel.y = raw[1]/1000.0f;
        chassis->kinematic.target_vel.w = raw[2]/1000.0f;
        chassis->yaw_100 = (msg[0].Data[6]<<8 | msg[0].Data[7]);
    }
    if(recv_flag[1]) {                          //0x102
        chassis->state.sup_power_flag = msg[1].Data[0];
        chassis->state.chassis_mode = msg[1].Data[1];
        chassis->pitch_100 = (msg[1].Data[2]<<8 | msg[1].Data[3]);
        chassis->state.gimbal_mode = msg[1].Data[4];
        chassis->state.auto_fire = (msg[1].Data[5]>>0)&0x01;
        chassis->state.laser_flag = (msg[1].Data[5]>>1)&0x01;
        chassis->state.graphic_init_flag = (msg[1].Data[5]>>2)&0x01;
        chassis->state.fire_freq = (msg[1].Data[5]>>3)&0x01;
        chassis->state.fric_flag = (msg[1].Data[5]>>4)&0x01;
        chassis->state.enemy_id = (msg[1].Data[5]>>5)&0x07;
    }

}

// void recvGimData(Chassis_t *chassis) {       //链表版本
//     uint8_t recv_flag[2] = {0};
//     CanMsgNode_t *node, *prev_node;
//     CanMsgList_t *p = chassis->gimbal_canRx;
//     xSemaphoreTake(p->mutex, portMAX_DELAY);
//     //从后向前遍历（因为新消息被添加到尾部）
//     node = p->tail;
//     while(node != NULL) {
//         uint32_t id = node->msg.StdId;
//         if(!recv_flag[id-0x101]) {
//             recv_flag[id-0x101] = 1;
//             switch(id) {
//                 case 0x101:
//                     chassis->kinematic.target_vel.x = (node->msg.Data[0]<<8 | node->msg.Data[1]);
//                     chassis->kinematic.target_vel.y = (node->msg.Data[2]<<8 | node->msg.Data[3]);
//                     chassis->kinematic.target_vel.w = (node->msg.Data[4]<<8 | node->msg.Data[5]);
//                     chassis->yaw_100 = (node->msg.Data[6]<<8 | node->msg.Data[7]);
//                     break;
//                 case 0x102:
//                     chassis->state.sup_power_flag = node->msg.Data[0];
//                     chassis->state.chassis_mode = node->msg.Data[1];
//                     chassis->pitch_100 = (node->msg.Data[2]<<8 | node->msg.Data[3]);
//                     chassis->state.gimbal_mode = node->msg.Data[4];
//                     chassis->state.auto_fire = (node->msg.Data[5]>>0)&0x01;
//                     chassis->state.laser_flag = (node->msg.Data[5]>>1)&0x01;
//                     chassis->state.graphic_init_flag = (node->msg.Data[5]>>2)&0x01;
//                     chassis->state.fire_freq = (node->msg.Data[5]>>3)&0x01;
//                     chassis->state.fric_flag = (node->msg.Data[5]>>4)&0x01;
//                     chassis->state.enemy_id = (node->msg.Data[5]>>5)&0x07;
//                     break;
//                 default:
//                     break;
//             }
//         }
//         prev_node = node->prev;
//         vPortFree(node);
//         node = prev_node;
//     }
//     xSemaphoreGive(p->mutex);
//     return;
// }

/**
 * @brief  底盘运动控制
 * @param  Chassis_t *chassis 底盘结构体指针
 * @retval None
*/
void moveCtrl(Chassis_t *chassis) {
    uint8_t i;
    Kinematic_t *k = &(chassis->kinematic);//这里别忘了用指针，一开始给忘了
    updateWheels(&(chassis->kinematic));
		invKinematic(&(chassis->kinematic));
    for(i=0; i<4; i++) {
        k->motor[i].target_current = 
            PID_Calc(&(k->pid_speed[i]), k->motor[i].target_speed, k->motor[i].speed);
    }
    setMotorCurrent(k);
}

// /**********************************************************************************************************
// *函 数 名: BuildF105
// *功能说明: 构建要传给上层板的F105结构体
// *形    参: 无
// *返 回 值: 无
// **********************************************************************************************************/
// void BuildF105(void)
// {
	
// 	if(JudgeReceive.robot_id < 10)
// 		F105.Sendmessage.RobotRed = 1;
// 	else
// 		F105.Sendmessage.RobotRed = 0;			//0为蓝色，1为红色
// 	switch(JudgeReceive.BulletSpeedMax17)
// 	{
// 		case 15:
// 		{
// 			F105.Sendmessage.BulletSpeedLevel = 0;
// 			break;
// 		}
// 		case 18:
// 		{
// 			F105.Sendmessage.BulletSpeedLevel = 1;
// 			break;
// 		}
// 		case 30:
// 		{
// 			F105.Sendmessage.BulletSpeedLevel = 2;
// 			break;
// 		}
// 		default:
// 		{
// 			F105.Sendmessage.BulletSpeedLevel = 0;
// 			break;
// 		}
// 	}
// 	F105.Sendmessage.shooterHeat17 = JudgeReceive.shooterHeat17;
// 	F105.Sendmessage.HeatMax17 = JudgeReceive.HeatMax17;
// 	F105.Sendmessage.HeatCool17 = (unsigned char)(JudgeReceive.HeatCool17);
// 	F105.Sendmessage.RobotLevel = JudgeReceive.RobotLevel;
	
// 	F105.ChassisSpeedw=0.026f*(chassis.ChassisMotorCanReceive[0].RealSpeed+chassis.ChassisMotorCanReceive[1].RealSpeed+chassis.ChassisMotorCanReceive[2].RealSpeed+chassis.ChassisMotorCanReceive[3].RealSpeed);
// }
/**********************************************************************************************************
*º¯ Êý Ãû: Can2Send0
*¹¦ÄÜËµÃ÷: can2·¢ËÍº¯Êý
*ÐÎ    ²Î: ChassisSpeedw, Remain_power, IsShootAble, RobotRed, BulletSpeedLevel
*·µ »Ø Öµ: ÎÞ
**********************************************************************************************************/
// void Can2Send0(F105_Typedef *F105_Send)
// {
// 	CanTxMsg tx_message;
// 	tx_message.IDE = CAN_ID_STD;    
// 	tx_message.RTR = CAN_RTR_DATA; 
// 	tx_message.DLC = 0x02;    
// 	tx_message.StdId = 0x100;
	
// 	memcpy(&tx_message.Data[0],&F105_Send->ChassisSpeedw,2);
// 	CAN_Transmit(CAN2,&tx_message);
// }
/**
 * @brief  通过分配的can接口发送云台数据
 * @param  Chassis_t *chassis 底盘结构体指针
 * @retval None
*/
void sendGimData(Chassis_t *chassis) {
    short w = (short)chassis->kinematic.target_vel.w*1000;
    CanTxMsg tx_msg;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x02;
    tx_msg.StdId = 0x100;
    tx_msg.Data[0] = (w>>8)&0xff;
    tx_msg.Data[1] = w&0xff;
    CAN_Transmit(CAN2, &tx_msg);
    return;
}

