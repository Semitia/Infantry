#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__

#include "main.h"
#include "chassis.h"

#define Vol2Current(x,y) (((x) - (y)*76.0)/1.35f) //输入（电压，转速），输出对应的电流
#define CAP_MAX_W      7000
#define Rand_S         0.5f   //周期长短
#define RandThreshold  0.4f   //直流偏置
#define RANDA          1.3f   //正弦幅值

void Chassis_task(void *pvParameters);

#endif
