#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdfloat.h"

//电机参数宏定义
#define RM3508_CURRENT_RATIO (20.0/16384.0)  
#define RM3508_RPM_RAD (PI/30)

#define RM6020_CURRENT_RATIO (20.0/25000.0)  
#define RM6020_RPM_RAD (PI/30)

#define GM6020_MAX_VOLTAGE 30000 //最大发送电压值,最大值为30000
#define GM6020_MIN_VOLTAGE -GM6020_MAX_VOLTAGE

#define GM6020_MAX_CURRENT 16000
#define GM6020_MIN_CURRENT -GM6020_MAX_CURRENT
#define Vol2Current(x,y) (((x) - (y)*76.0)/1.35f) //输入（电压，转速），输出对应的电流

#define RM3508_R 0.1231
#define RM3508_K 0.001685
#define RM3508_P0 12.75
#define RM3508_K_MAX 0.0017
#define RM3508_K_MIN 0.00145
#define CAP_C (55/9)  //超级电容组的容值

/**
 * 速度结构体
*/
typedef struct __Vel_t{
	float32_t v;        //m/s
	float32_t yaw;      //rad
}Vel_t;

//底盘类型
typedef enum __ChassisTypeEnum{
    MECANUM_WHEEL,  // 麦克纳母轮
    OMNI_WHEEL,     // 全向轮
    STEERING_WHEEL  // 舵轮
}ChassisTypeEnum

typedef struct __Chassis_t {
    ChassisTypeEnum type_enum;

}Chassis_t;

void chassisInit(Chassis_t *c, ChassisTypeEnum type_enum);


#endif