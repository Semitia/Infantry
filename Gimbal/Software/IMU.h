#ifndef __IMU_H
#define __IMU_H

#include "can2.h"
#include "myMath.h"
#include "counter.h"
#define IMU_CAN_RX &can2_rx1

#define GRAVITY 9.78F                       //重力加速度
#define GYRO_SCALE PI/16.3835F/ 180         
#define ACC_SCALE GRAVITY/8192.0F
#define G_NORM_MAX_MIN_DIFF_THRESH 0.2f     //重力加速度模值最大最小差异阈值
#define GYRO_DIFF_THRESH 0.35f              //陀螺仪数据差异阈值
#define GYRO_OFFSET_MAX 0.2f                //陀螺仪零偏最大值
#define IF_CALIBRATE 1                      //是否实时标定
#define MAX_CALIB_TIMES 6000
#define MIN_CALIB_TIMES 4000

#define GX_OFF  0.00887970906f
#define GY_OFF  0.00248288154f
#define GZ_OFF -0.00579680875f
#define G_NORM  9.6937418f

/**
 * @brief IMU结构体
*/
typedef struct __IMU_t {
    float g_norm;                //重力加速度模值
    float gyro[3];               //角速度
    float gyro_offset[3];        //陀螺仪零偏
    float acc[3];                //加速度
    float acc_scale;             //加速度矫正比例

    CanRing_t *can_rx;
}IMU_t;

void imuInit(IMU_t *imu);
void recvData(IMU_t *imu);
void updateIMU(IMU_t *imu);
void calibrateIMU(IMU_t *imu);
#endif

