/**********************************************************************************************************
 * @文件     motor.c
 * @说明     电机封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-11
 **********************************************************************************************************/

#include "motor.h"

void motorInit(Motor_t *m, MotorTypeEnum type, uint8_t id) {
    lowPassInit(&(m->lp_spd), K_WHEEL);
    lowPassInit(&(m->lp_cut), K_CURRENT);

    m->type_enum = type;
    m->id = id;
    m->temperature = 0;
    m->speed = 0;
    m->last_speed = 0;
    m->current = 0;
    m->angle = 0;
    m->last_angle = 0;
    m->revolutions = 0;
    switch(type) {
        case RM2006:
            // m->current_ratio = RM2006_CURRENT_RATIO;
            // m->rpm_rad = RM2006_RPM_RAD;
            // m->max_voltage = RM2006_MAX_VOLTAGE;
            // m->min_voltage = RM2006_MIN_VOLTAGE;
            break;
        case RM3508:
            m->current_ratio = RM3508_CURRENT_RATIO;
            m->rpm_rad = RM3508_RPM_RAD/RM3508_REDUC_RATIO;
            // m->max_voltage = RM3508_MAX_VOLTAGE;
            // m->min_voltage = RM3508_MIN_VOLTAGE;
            break;
        case RM6020:
            m->current_ratio = RM6020_CURRENT_RATIO;
            m->rpm_rad = RM6020_RPM_RAD;
            m->max_voltage = GM6020_MAX_VOLTAGE;
            m->max_current = GM6020_MAX_CURRENT;
            break;
        default:
            break;
    }
    return;
}

void motorUpdate(Motor_t *m, uint8_t *data) {
    //注意使用int16_t来转换数据类型
    float raw_speed = ((int16_t)(data[2] << 8) | data[3]) * m->rpm_rad;                 //转子rpm转换到轮子rad/s
    float raw_current = ((int16_t)(data[4] << 8) | data[5]) * m->current_ratio;         //电流转换到A
    m->speed = lowPass(&(m->lp_spd), raw_speed);
    m->current = lowPass(&(m->lp_cut), raw_current);
    return;
}


void motorUpdateAll(Motor_t *m, uint8_t *data) {
    uint16_t raw_angle = (data[0] << 8) | data[1];							
    motorUpdate(m, data);
    m->temperature = data[6];
    m->angle = raw_angle / 8191.0f * 360.0f;
    return;
}

