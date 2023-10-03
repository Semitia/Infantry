#ifndef __IMU_H
#define __IMU_H

#include "can2.h"
#include "myMath.h"
#define IMU_CAN_RX &can2_rx1

#define GRAVITY 9.78F //重力加速度
#define GYRO_SCALE PI/16.3835F/ 180
#define ACC_SCALE GRAVITY/8192.0F

typedef struct __IMU_t {
    float gyro[3];      //
    float acc[3];       //加速度

    CanRing_t *can_rx;
}IMU_t;

void imuInit(IMU_t *imu);
void recvData(IMU_t *imu);
#endif

