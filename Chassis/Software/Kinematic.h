#ifndef __KINEMATIC_H
#define __KINEMATIC_H

#include "can1.h"
#include "pid.h"
#include "motor.h"

//底盘类型
#define CHASSIS_TYPE 1 //0 全向轮，1 麦克纳母轮，2 舵轮

//车盘尺寸宏定义
#define WHEEL_RADIUS 0.1f               //轮子半径
#define CHASSIS_RADIUS 0.28f            //车盘半径


/**
 * 速度结构体
*/
typedef struct __Velocity_t {
    float x;    // x方向速度 m/s
    float y;    // y方向速度 m/s
    float w;    // 角速度   rad/s
}Velocity_t;

/**
 * 运动学结构体
 * 理论上来说，电机控制接口应该是与电机封装在一起的，但是我们用的都是Can通信电机，
 * 发送一次信息可以控制多个电机，所以这里就和can绑定了
*/
typedef struct __Kinematic_t {    
    Velocity_t real_vel;                            //实际速度
    Velocity_t target_vel;                          //目标速度

    Motor_t motor[4];                               //电机
    Pid_t pid_speed[4];                             //速度环pid
    Pid_t pid_angle[4];                             //角度环pid

    Motor_t steering_motor[4];                      //舵轮转向电机
    Pid_t steering_pid_angle[4];                    //舵轮转向角度环pid
    Pid_t steering_pid_speed[4];                    //舵轮转向速度环pid

    // CanMsgList_t *can_datalist;                     //can数据接收链表,用于更新电机数据
    // CanMsgList_t *can_datalist_steering;            //舵轮额外需要4个电机数据
    CanRing_t *can_ring;                            
    CanRing_t *can_ring_steering;                   
    CAN_TypeDef *can_tx;                            //can发送接口

} Kinematic_t;

void kinematicInit(Kinematic_t *kinematic, CAN_TypeDef *can_tx, CanRing_t *can_ring);
void forKinematic(Kinematic_t *kinematic);
void invKinematic(Kinematic_t *kinematic);
void updateWheels(Kinematic_t *kinematic);
void setMotorCurrent(Kinematic_t *kinematic);
void setMotorCurTest(short a, short b, short c, short d, CAN_TypeDef *can_tx);

#endif

