#include "Chassis.h"

/**
 * @brief  底盘初始化
 * @param  Chassis_t *chassis 底盘结构体指针
 * @param  ChassisTypeEnum type_enum 底盘类型枚举
 * @retval None
*/
void chassisInit(Chassis_t *chassis, ChassisTypeEnum type_enum) {
    int i=0;
    chassis->type_enum = type_enum;
    chassis->real_vel.x = 0;
    chassis->real_vel.y = 0;
    chassis->real_vel.w = 0;
    for(i=0; i<4; i++) {
        motorInit(&(chassis->motor[i]), RM3508, i);
        chassis->pid_speed[i].deadband = 0;
        chassis->pid_speed[i].P        = 20.0f;
        chassis->pid_speed[i].I        = 1.0f;
        chassis->pid_speed[i].D        = 0.0f;
        chassis->pid_speed[i].ErrorMax = 1000.0f;
        chassis->pid_speed[i].IMax     = 2500.0f;
        chassis->pid_speed[i].SetPoint = 0.0f;
        chassis->pid_speed[i].OutMax   = 16000.0f;


    }
    switch(type_enum) {
        case MECANUM_WHEEL:
            chassis->forKinematic = mecanumForKinematic;
            chassis->invKinematic = mecanumInvKinematic;
            chassis->update = update_4wheels;
            break;
        case OMNI_WHEEL:
            chassis->forKinematic = omniForKinematic;
            chassis->invKinematic = omniInvKinematic;
            break;
        case STEERING_WHEEL:
            chassis->forKinematic = steeringForKinematic;
            chassis->invKinematic = steeringInvKinematic;
            //舵轮底盘有8个电机
            for(i=4; i<8; i++) {
                motorInit(&(chassis->motor[i]), RM3508, i);
                chassis->pid_speed[i].deadband = 0;
                chassis->pid_speed[i].P        = 120.0f;
                chassis->pid_speed[i].I        = 0.2f;
                chassis->pid_speed[i].D        = 0.0f;
                chassis->pid_speed[i].ErrorMax = 8000.0f;
                chassis->pid_speed[i].IMax     = 16000.0f;
                chassis->pid_speed[i].SetPoint = 0.0f;
                chassis->pid_speed[i].OutMax   = 30000.0f;

                chassis->pid_angle[i].deadband = 2.5f;
                chassis->pid_angle[i].P        = 7.5f;
                chassis->pid_angle[i].I        = 0.0f;
                chassis->pid_angle[i].D        = 0.0f;
                chassis->pid_angle[i].ErrorMax = 1000.0f;
                chassis->pid_angle[i].IMax     = 1000.0f;
                chassis->pid_angle[i].SetPoint = 0.0f;
                chassis->pid_angle[i].OutMax   = 4000.0f;

                // chassis.SteeringAnglePosition_init[0] = 5147;
                // chassis.SteeringAnglePosition_init[1] = 3061;
                // chassis.SteeringAnglePosition_init[2] = 4921;
                // chassis.SteeringAnglePosition_init[3] = 326;

            }
            break;
        default:
            break;
    }
    return;
}

/**
 * @brief  麦克纳母轮底盘前向运动学解算
 * @param  Chassis_t *chassis 底盘结构体指针
 * @retval None
 * 
*/
void mecanumForKinematic(Chassis_t *chassis) {
    chassis->real_vel.x = (chassis->motor[0].speed + chassis->motor[1].speed + chassis->motor[2].speed + chassis->motor[3].speed);
    chassis->real_vel.y = (chassis->motor[0].speed - chassis->motor[1].speed + chassis->motor[2].speed - chassis->motor[3].speed);
    chassis->real_vel.w = (chassis->motor[0].speed - chassis->motor[1].speed - chassis->motor[2].speed + chassis->motor[3].speed);
}

/**
 * @brief  麦克纳母轮底盘逆向运动学解算
 * @param  Chassis_t *chassis 底盘结构体指针
 * @retval None
 * 
*/
void mecanumInvKinematic(Chassis_t *chassis) { 
    chassis->motor[0].target_speed = + chassis->target_vel.x + chassis->target_vel.y - chassis->target_vel.w;
    chassis->motor[1].target_speed = - chassis->target_vel.x + chassis->target_vel.y - chassis->target_vel.w;
    chassis->motor[2].target_speed = + chassis->target_vel.x - chassis->target_vel.y - chassis->target_vel.w;
    chassis->motor[3].target_speed = - chassis->target_vel.x - chassis->target_vel.y - chassis->target_vel.w;
}
