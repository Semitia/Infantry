/**********************************************************************************************************
 * @文件     motor.c
 * @说明     电机封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-11
 **********************************************************************************************************/

#include "motor.h"

void motorInit(Motor_t *m, uint8_t id) {
    m->id = id;
    m->temperature = 0;
    m->raw_angle = 0;
    m->speed = 0;
    m->current = 0;
    m->angle = 0;
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