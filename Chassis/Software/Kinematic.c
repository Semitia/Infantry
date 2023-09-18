/**********************************************************************************************************
 * @文件     Kinematic.c
 * @说明     常用底盘运动学封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-16
 **********************************************************************************************************/
#include "Kinematic.h"

#if CHASSIS_TYPE == 0 //全向轮
/**
 * @brief 全向轮底盘正解
 * @param kinematic 运动学结构体
 * @retval None
 */
void forKinematic(Kinematic_t *kinematic) {
    
    return;
}


void invKinematic(Kinematic_t *kinematic) {

    return;
}

#elif CHASSIS_TYPE == 1 //麦克纳母轮

void kinematicInit(Kinematic_t *kinematic) {
    int i=0;
    kinematic->real_vel.x = 0;
    kinematic->real_vel.y = 0;
    kinematic->real_vel.w = 0;
    for(i=0; i<4; i++) {
        motorInit(&(kinematic->motor[i]), RM3508, i);
        kinematic->pid_speed[i].deadband = 0;
        kinematic->pid_speed[i].P        = 20.0f;
        kinematic->pid_speed[i].I        = 1.0f;
        kinematic->pid_speed[i].D        = 0.0f;
        kinematic->pid_speed[i].ErrorMax = 1000.0f;
        kinematic->pid_speed[i].IMax     = 2500.0f;
        kinematic->pid_speed[i].SetPoint = 0.0f;
        kinematic->pid_speed[i].OutMax   = 16000.0f;
    }
    return;
}

void forKinematic(Kinematic_t *kinematic) {
    kinematic->real_vel.x = kinematic->motor[0].speed + kinematic->motor[1].speed + kinematic->motor[2].speed + kinematic->motor[3].speed;
    kinematic->real_vel.y = -kinematic->motor[0].speed + kinematic->motor[1].speed + kinematic->motor[2].speed - kinematic->motor[3].speed;
    kinematic->real_vel.w = (kinematic->motor[0].speed - kinematic->motor[1].speed + kinematic->motor[2].speed - kinematic->motor[3].speed) * 0.25f;
    return;
}

void invKinematic(Kinematic_t *kinematic) {
    kinematic->motor[0].target_speed = + kinematic->target_vel.x + kinematic->target_vel.y - kinematic->target_vel.w;
    kinematic->motor[1].target_speed = - kinematic->target_vel.x + kinematic->target_vel.y - kinematic->target_vel.w;
    kinematic->motor[2].target_speed = + kinematic->target_vel.x - kinematic->target_vel.y - kinematic->target_vel.w;
    kinematic->motor[3].target_speed = - kinematic->target_vel.x - kinematic->target_vel.y - kinematic->target_vel.w;
    return;
}

#elif CHASSIS_TYPE == 2 //舵轮
void kinematicInit(Kinematic_t *kinematic) {
    int i=0;
    kinematic->real_vel.x = 0;
    kinematic->real_vel.y = 0;
    kinematic->real_vel.w = 0;
    for(i=0; i<4; i++) {
        motorInit(&(kinematic->motor[i]), RM3508, i);
        kinematic->pid_speed[i].deadband = 0;
        kinematic->pid_speed[i].P        = 20.0f;
        kinematic->pid_speed[i].I        = 1.0f;
        kinematic->pid_speed[i].D        = 0.0f;
        kinematic->pid_speed[i].ErrorMax = 1000.0f;
        kinematic->pid_speed[i].IMax     = 2500.0f;
        kinematic->pid_speed[i].SetPoint = 0.0f;
        kinematic->pid_speed[i].OutMax   = 16000.0f;

        motorInit(&(kinematic->steering_motor[i]), RM3508, i+4);
        kinematic->steering_pid_speed[i].deadband = 0;
        kinematic->steering_pid_speed[i].P        = 120.0f;
        kinematic->steering_pid_speed[i].I        = 0.2f;
        kinematic->steering_pid_speed[i].D        = 0.0f;
        kinematic->steering_pid_speed[i].ErrorMax = 8000.0f;
        kinematic->steering_pid_speed[i].IMax     = 16000.0f;
        kinematic->steering_pid_speed[i].SetPoint = 0.0f;
        kinematic->steering_pid_speed[i].OutMax   = 30000.0f;

        kinematic->steering_pid_angle[i].deadband = 2.5f;
        kinematic->steering_pid_angle[i].P        = 7.5f;
        kinematic->steering_pid_angle[i].I        = 0.0f;
        kinematic->steering_pid_angle[i].D        = 0.0f;
        kinematic->steering_pid_angle[i].ErrorMax = 1000.0f;
        kinematic->steering_pid_angle[i].IMax     = 1000.0f;
        kinematic->steering_pid_angle[i].SetPoint = 0.0f;
        kinematic->steering_pid_angle[i].OutMax   = 4000.0f;
        
    }
    return;
}

void forKinematic(Kinematic_t *kinematic) {

    return;
}

void invKinematic(Kinematic_t *kinematic) {

    return;
}

#endif