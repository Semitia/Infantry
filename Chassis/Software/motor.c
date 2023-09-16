/**********************************************************************************************************
 * @文件     motor.c
 * @说明     电机封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-11
 **********************************************************************************************************/

#include "motor.h"

void motorInit(Motor_t *m, MotorTypeEnum type, uint8_t id) {
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
            m->rpm_rad = RM3508_RPM_RAD;
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
    m->speed = (data[2] << 8) | data[3];
    m->current = (data[4] << 8) | data[5];
    return;
}

void motorUpdateAll(Motor_t *m, uint8_t *data) {
    m->raw_angle = (data[0] << 8) | data[1];
    m->speed = (data[2] << 8) | data[3];
    m->current = (data[4] << 8) | data[5];
    m->temperature = data[6];
    m->angle = m->raw_angle / 8191.0f * 360.0f;
    return;
}