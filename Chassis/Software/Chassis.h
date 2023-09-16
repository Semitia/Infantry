#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"
#include "pid.h"
#include "can1.h"
#include "Kinematic.h"

#define CAP_C (55/9)  //超级电容组的容值

/**
 * 底盘类型枚举
*/
typedef enum __ChassisTypeEnum{
    MECANUM_WHEEL,  // 麦克纳母轮
    OMNI_WHEEL,     // 全向轮
    STEERING_WHEEL  // 舵轮
}ChassisTypeEnum


/**
 * 底盘运动控制结构体
 * 可以配置为麦克纳母轮、全向轮、舵轮，对应不同的运动解算控制结构体
*/
typedef struct __Chassis_t {
    ChassisTypeEnum type_enum;                      //底盘类型
    Velocity_t real_vel;                            //实际速度
    Velocity_t target_vel;                          //目标速度

    Motor_t motor[8];                               //电机
    Pid_Typedef pid_speed[8];                       //速度环pid
    Pid_Typedef pid_angle[8];                       //角度环pid
    /*舵轮相关
    ......
    */
    void (*forKinematic)(struct __Chassis_t *chassis);    //forward kinematic 前向运动学函数指针
    void (*invKinematic)(struct __Chassis_t *chassis);    //inverse kinematic 逆向运动学函数指针
    void (*update)(struct __Chassis_t *chassis);          //底盘更新函数指针
}Chassis_t;

void chassisInit(Chassis_t *chassis, ChassisTypeEnum type_enum);
void update_4wheels(Chassis_t *chassis, CanMsgList_t *list);

#endif