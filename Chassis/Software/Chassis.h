#ifndef CHASSIS_H
#define CHASSIS_H

#include "Kinematic.h"

#define CAP_C (55/9)  //超级电容组的容值

/**
 * 底盘类型枚举
*/
typedef enum __ChassisTypeEnum{
    MECANUM_WHEEL,  // 麦克纳母轮
    OMNI_WHEEL,     // 全向轮
    STEERING_WHEEL  // 舵轮
}ChassisTypeEnum;

/**
 * 底盘运动控制结构体
 * 可以配置为麦克纳母轮、全向轮、舵轮，对应不同的运动解算控制结构体
*/
typedef struct __Chassis_t {
    ChassisTypeEnum type_enum;                      //底盘类型
    Kinematic_t kinematic;                          //运动控制结构体

    
}Chassis_t;

void chassisInit(Chassis_t *chassis, ChassisTypeEnum type_enum);

#endif

