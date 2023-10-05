#ifndef __SHOOT_H
#define __SHOOT_H

#include "CanRing.h"
#include "motor.h"

#define FRIC_MOTOR0_ID 0x202
#define FRIC_MOTOR1_ID 0x201
#define PLUCK_MOTOR_ID 0x204

typedef struct __Shoot_t {
	int a;
} Shoot_t;

void sendSingleMoto(Motor_t *moto, CAN_TypeDef* CANx);
#endif

