#include "IMU.h"

void imuInit(IMU_t *imu) {
    return;
}

/**
 * @brief 通过Can接口接收IMU数据
 * @param imu IMU结构体
 * @note 0x100 short类型
 *       0x101 线加速度信息
*/
void recvData(IMU_t *imu) {
    uint8_t recv_flag[2] = {0};
    CanRxMsg msg[2];
    CanRing_t *p = imu->can_rx;
    while(canRingHasMsg(p)) {
        CanRxMsg temp_msg = popCanMsg(p);
        msg[temp_msg.StdId-0x100] = temp_msg;
        recv_flag[temp_msg.StdId-0x100] = 1;
    }
    if(recv_flag[0]) { 
        imu->gyro[0] = ((short)(msg[0].Data[0]<<8 | msg[0].Data[1])) * GYRO_SCALE;
        imu->gyro[1] = ((short)(msg[0].Data[2]<<8 | msg[0].Data[3])) * GYRO_SCALE;
        imu->gyro[2] = ((short)(msg[0].Data[4]<<8 | msg[0].Data[5])) * GYRO_SCALE;
    }
    if(recv_flag[1]) {
        imu->acc[0] = ((short)(msg[1].Data[0]<<8 | msg[1].Data[1])) * ACC_SCALE;
        imu->acc[1] = ((short)(msg[1].Data[2]<<8 | msg[1].Data[3])) * ACC_SCALE;
        imu->acc[2] = ((short)(msg[1].Data[4]<<8 | msg[1].Data[5])) * ACC_SCALE;
    }
    
    return;
}

