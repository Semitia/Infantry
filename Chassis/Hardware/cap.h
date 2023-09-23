#ifndef _CAP_H
#define _CAP_H

#define Bat_on GPIO_SetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x2)
#define Bat_off GPIO_ResetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x2)
#define CAP_on GPIO_SetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x1)
#define CAP_off GPIO_ResetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x1)
#define Charge_On GPIO_SetBits(POWER_CHARGE_GPIOx, POWER_CHARGE_GPIO_Pin_x1)
#define Charge_Off GPIO_ResetBits(POWER_CHARGE_GPIOx, POWER_CHARGE_GPIO_Pin_x1)

#include "main.h"

#endif

