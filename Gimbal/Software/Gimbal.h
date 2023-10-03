#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "RemoteCtrl.h"

#define MIN_SPEED -1.0                             //速度最小值
#define MAX_SPEED 1.0                              //速度最大值

/**
 * 速度结构体
*/
typedef struct __Velocity_t {
    float x;    // x方向速度 m/s
    float y;    // y方向速度 m/s
    float w;    // 角速度   rad/s
}Velocity_t;

/**
 * @brief 云台结构体
*/
typedef struct __Gimbal_t {
    Velocity_t vel;             //速度 云台坐标系

    RC_t rc;                    //遥控器结构体


}Gimbal_t;

void gimbalInit(Gimbal_t *gimbal);
void gimUpdate(Gimbal_t *gimbal);

#endif

