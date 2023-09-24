#ifndef CHASSIS_H
#define CHASSIS_H

#include "Kinematic.h"
#include "can2.h"

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
 * 状态结构体
*/
typedef struct __StateFlag_t {
    uint8_t sup_power_flag;     //0: 超级电容关闭
    uint8_t chassis_mode;       //底盘当前模式
    uint8_t gimbal_mode;        //云台当前模式
    uint8_t auto_fire;          //0: 手动发射，1: 自动发射
    uint8_t fire_freq;          //0: 正常，1: 高频
    uint8_t fric_flag;          //0: 摩擦轮关闭，1: 摩擦轮开启?猜的?
    uint8_t laser_flag;         //0: 激光关闭，1: 激光开启
    uint8_t graphic_init_flag;  //0: 图传未初始化，1: 图传已初始化
    uint8_t enemy_id;           //敌方id
}StateFlag_t;


/**
 * 底盘运动控制结构体
 * 可以配置为麦克纳母轮、全向轮、舵轮，对应不同的运动解算控制结构体
*/
typedef struct __Chassis_t {
    ChassisTypeEnum type_enum;                      //底盘类型
    StateFlag_t state;                              //状态
    Kinematic_t kinematic;                          //运动控制结构体

    CanMsgList_t *gimbal_canRx;                     //云台接收数据链表
    
    short pitch_100;                                //云台pitch角度*100
    short yaw_100;                                  //云台yaw角度*100
}Chassis_t;

void chassisInit(Chassis_t *chassis, ChassisTypeEnum type_enum);
void recvGimData(Chassis_t *chassis);
#endif

