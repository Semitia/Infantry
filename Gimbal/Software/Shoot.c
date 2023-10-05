#include "Shoot.h"

/**
 * @brief 根据电机ID发送单个电机的CAN数据
 * @param moto 电机结构体
 * @param CANx CAN端口
*/
void sendSingleMoto(Motor_t *moto, CAN_TypeDef* CANx) {
    uint16_t min_id;
    CanTxMsg tx_msg;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    if(moto->id >= 0x205) {
        min_id = 0x205;
        tx_msg.StdId = 0x1FF;
    } else {
        min_id = 0x201;
        tx_msg.StdId = 0x200;
    }
    tx_msg.Data[moto->id - min_id] = (uint8_t)((moto->target_current >> 8) & 0xFF);
    tx_msg.Data[moto->id - min_id + 1] = (uint8_t)(moto->target_current & 0xFF);
    CAN_Transmit(CANx, &tx_msg);
}
