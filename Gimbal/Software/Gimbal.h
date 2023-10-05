#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "RemoteCtrl.h"
#include "INS.h"
#include "Shoot.h"
#include "motor.h"
#include "pid.h"

#define MIN_SPEED -1.0                             //速度最小值
#define MAX_SPEED 1.0                              //速度最大值
#define MIN_PITCH  0.0f                           //pitch轴最小角度
#define MAX_PITCH 40*PI/180                 //pitch轴最大角度
#define MIN_YAW  -PI                         //yaw轴最小角度
#define MAX_YAW  PI                          //yaw轴最大角度

#define YAW_ID 0x205                               //yaw轴电机id
#define PITCH_ID 0x207                             //pitch轴电机id
#define YAW_CAN_TX CAN1
#define YAW_CAN_RX &can1_rx1
#define PITCH_CAN_TX CAN2
#define PITCH_CAN_RX &can2_rx0

typedef struct __ChassisInfo_t {
    /* Rx */
    short HeatMax17;                               //17mm枪口最大热量
    short shooterHeat17;                           //17mm枪口热量
    unsigned char RobotRed;                        //红蓝方机器人
    unsigned char HeatCool17;                      //17mm枪口冷却
    unsigned char BulletSpeedLevel;                //子弹速度等级
    unsigned char RobotLevel;                      //机器人等级
    /* Tx */

}ChassisInfo_t;

/**
 * 速度结构体
*/
typedef struct __Velocity_t {
    float x;    // x方向速度 m/s
    float y;    // y方向速度 m/s
    float w;    // 角速度   rad/s
}Velocity_t;

/**
 * @brief 云台位姿控制结构体
*/
typedef struct __GimPosture_t {
    float tar_yaw;              //目标yaw轴角度
    float tar_pitch;            //目标pitch轴角度
    Motor_t moto_yaw;           //yaw轴电机
    Motor_t moto_pitch;         //pitch轴电机
    Pid_Typedef yaw_spd_pid;    //yaw轴速度pid
    Pid_Typedef pitch_spd_pid;  //pitch轴速度pid
    Pid_Typedef yaw_pos_pid;    //yaw轴位置pid
    Pid_Typedef pitch_pos_pid;  //pitch轴位置pid
}GimPosture_t;

/**
 * @brief 云台结构体
*/
typedef struct __Gimbal_t {
    RC_t rc;                    //遥控器结构体
    INS_t ins;                  //惯导结构体
    Shoot_t shoot;              //射击结构体
    Velocity_t vel;             //速度 云台坐标系
    GimPosture_t pose;       //云台位姿控制结构体
}Gimbal_t;

void gimbalInit(Gimbal_t *gimbal);
void gimUpdate(Gimbal_t *gimbal);
void setPosCur(Gimbal_t *gimbal);
#endif

