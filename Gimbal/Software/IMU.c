#include "IMU.h"

/**
 * @brief 实时标定陀螺仪零偏
 * @param imu IMU结构体
*/
void calibrateIMU(IMU_t *imu) {
    static float startTime;
    float gyroMax[3] = {-10, -10, -10}, gyroMin[3] = {10, 10, 10}, gyroDiff[3];
    float gNormTemp, gNormMax = -10, gNormMin = 10, g_norm_diff;
    uint32_t caliCount = 0;
    uint16_t i;

    startTime = GetTime_s();
    do {
        //新一轮标定
        if (GetTime_s() - startTime > 10) {
            // 校准超时
            imu->gyro_offset[0] = GX_OFF;
            imu->gyro_offset[1] = GY_OFF;
            imu->gyro_offset[2] = GZ_OFF;
            imu->g_norm = G_NORM;
            break;
        }
        for(i = 0; i < 3; i++) {imu->gyro_offset[i] = 0;}
        imu->g_norm = 0;
        //采集数据
        for(i = 0; i<MIN_CALIB_TIMES; i++) {
            while(!canRingHasMsg(imu->can_rx));             //等待新消息
            recvData(imu);
            gNormTemp = sqrtf(  imu->acc[0] * imu->acc[0] +
                                imu->acc[1] * imu->acc[1] +
                                imu->acc[2] * imu->acc[2]);
            imu->g_norm += gNormTemp;
            for(uint8_t j = 0; j < 3; j++) {
                imu->gyro_offset[j] += imu->gyro[j];
            }
            caliCount++;
            // 记录数据极差
            gNormMax = MAX(gNormMax, gNormTemp);
            gNormMin = MIN(gNormMin, gNormTemp);
            for(uint8_t j = 0; j < 3; j++) {
                gyroMax[j] = MAX(gyroMax[j], imu->gyro[j]);
                gyroMin[j] = MIN(gyroMin[j], imu->gyro[j]);
            }
        }
        g_norm_diff = gNormMax - gNormMin;
        for (uint8_t j = 0; j < 3; j++) {gyroDiff[j] = gyroMax[j] - gyroMin[j];}
        imu->g_norm /= (float)caliCount;
        for (uint8_t i = 0; i < 3; i++) {imu->gyro_offset[i] /= (float)caliCount;}
    }
    while ( g_norm_diff > G_NORM_MAX_MIN_DIFF_THRESH ||
            fabsf(imu->g_norm - GRAVITY) > 0.5f ||
            gyroDiff[0] > GYRO_DIFF_THRESH ||
            gyroDiff[1] > GYRO_DIFF_THRESH ||
            gyroDiff[2] > GYRO_DIFF_THRESH ||
            fabsf(imu->gyro_offset[0]) > GYRO_OFFSET_MAX ||
            fabsf(imu->gyro_offset[1]) > GYRO_OFFSET_MAX ||
            fabsf(imu->gyro_offset[2]) > GYRO_OFFSET_MAX );
    imu->acc_scale = imu->g_norm/GRAVITY;
    return;
}

/**
 * @brief 初始化IMU结构体
 * @param imu IMU结构体
*/
void imuInit(IMU_t *imu) {
	imu->can_rx = IMU_CAN_RX;
    if(IF_CALIBRATE) {
        calibrateIMU(imu);
    }
    else {
        imu->gyro_offset[0] = GX_OFF;
        imu->gyro_offset[1] = GY_OFF;
        imu->gyro_offset[2] = GZ_OFF;
        imu->g_norm = G_NORM;
        imu->acc_scale = G_NORM/GRAVITY;
    }
    return;
}


/**
 * @brief 通过Can接口接收IMU原始数据，为方便配合校准函数，没有进行校准
 * @param imu IMU结构体
 * @note 0x100 short类型
 *       0x101 线加速度信息
*/
void recvData(IMU_t *imu) {
    short raw[3];                               //记录原始数据，便于DEBUG
    CanRxMsg msg[2];
    CanRing_t *p = imu->can_rx;
    uint8_t recv_flag[2] = {0};
    while(canRingHasMsg(p)) {
        CanRxMsg temp_msg = popCanMsg(p);
        msg[temp_msg.StdId-0x100] = temp_msg;
        recv_flag[temp_msg.StdId-0x100] = 1;
    }
    if(recv_flag[0]) { 
        raw[0] = ((short)(msg[0].Data[1]<<8 | msg[0].Data[0]));
        raw[1] = ((short)(msg[0].Data[3]<<8 | msg[0].Data[2]));
        raw[2] = ((short)(msg[0].Data[5]<<8 | msg[0].Data[4]));
        imu->gyro[0] = raw[0] * GYRO_SCALE;
        imu->gyro[1] = raw[1] * GYRO_SCALE;
        imu->gyro[2] = raw[2] * GYRO_SCALE;
    }
    if(recv_flag[1]) {
        raw[0] = ((short)(msg[1].Data[1]<<8 | msg[1].Data[0]));
        raw[1] = ((short)(msg[1].Data[3]<<8 | msg[1].Data[2]));
        raw[2] = ((short)(msg[1].Data[5]<<8 | msg[1].Data[4]));
        imu->acc[0] = raw[0] * ACC_SCALE;
        imu->acc[1] = raw[1] * ACC_SCALE;
        imu->acc[2] = raw[2] * ACC_SCALE;
    }
    return;
}

void updateIMU(IMU_t *imu) {
    recvData(imu);
    for(uint8_t i = 0; i < 3; i++) {
        imu->gyro[i] -= imu->gyro_offset[i];
        imu->acc[i] *= imu->acc_scale;
    }
    return;
}

