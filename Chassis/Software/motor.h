#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "Mymath.h"
//电机参数宏定义

#define RM3508_CURRENT_RATIO (20.0/16384.0)         //电流转换比例
#define RM3508_RPM_RAD (PI/30)
#define RM3508_R 0.1231
#define RM3508_K 0.001685
#define RM3508_P0 12.75
#define RM3508_K_MAX 0.0017
#define RM3508_K_MIN 0.00145

#define RM6020_CURRENT_RATIO (20.0/25000.0)  
#define RM6020_RPM_RAD (PI/30)

#define GM6020_MAX_VOLTAGE 30000                    //最大发送电压值,最大值为30000
#define GM6020_MIN_VOLTAGE -GM6020_MAX_VOLTAGE

#define GM6020_MAX_CURRENT 16000
#define GM6020_MIN_CURRENT -GM6020_MAX_CURRENT

#define Vol2Current(x,y) (((x) - (y)*76.0)/1.35f)   //输入（电压，转速），输出对应的电流

typedef enum __MotorTypeEnum {
    RM2006,
    RM3508,
    RM6020
}MotorTypeEnum;


typedef struct __Motor_t {
    MotorTypeEnum type_enum;    //电机类型
    float current_ratio;        //电流转换比例
    float rpm_rad;              //rpm转换为rad/s的比例
    float max_voltage;          //最大电压值
    float max_current;          //最大电流值

    uint8_t id;
    uint8_t temperature;
    short speed;                //速度, 单位为rpm
    short last_speed;           //上一次的速度
    short target_speed;         //目标速度
    short current;              //电流, short类型为16位有符号整型
    short target_current;       //目标电流
    float angle;                //角度, 原始角度为0-8191，对应0-360度
    float last_angle;           //上一次的角度
    uint8_t revolutions;        //转数
}Motor_t;

void motorInit(Motor_t *m, MotorTypeEnum type, uint8_t id);
void motorUpdate(Motor_t *m, uint8_t *data);
void motorUpdateAll(Motor_t *m, uint8_t *data);


#endif

