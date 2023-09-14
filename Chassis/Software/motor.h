#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef struct __Motor_t {
    uint8_t id;
    uint8_t temperature;
    uint16_t raw_angle;
    short speed;
    short current;
    float angle;
}Motor_t;

void motorInit(Motor_t *m, uint8_t id);
void motorUpdate(Motor_t *m, uint8_t *data);
void motorUpdateAll(Motor_t *m, uint8_t *data);

#endif