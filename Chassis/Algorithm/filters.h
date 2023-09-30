#ifndef __FILTERS_H
#define __FILTERS_H

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 一阶低通滤波器
 * 滤波系数也可以根据时间间隔和时间常数计算得到，
 * 不过步兵代码中直接给出了这几个参数值，
 * 所以这里把两种方式对应的函数都写了一下。
*/
#define K_CHASSIS 0.01f             // 底盘速度滤波系数
#define K_CHASSIS_STEERING 0.03f    // 舵轮底盘速度滤波系数
#define K_WHEEL 0.1f                // 电机速度滤波系数
#define K_CURRENT 0.1f              // 电机电流滤波系数

typedef struct __LowPass_t {
    float tf;           // 时间常数
    float K;        // 滤波系数
    float last;         // 上一次的输出
    TickType_t ts;      // 上一次时间戳
} LowPass_t;

void lowPassInit(LowPass_t *lp, float K);
void lowPassInitTS(LowPass_t *lp, float tf);
float lowPass(LowPass_t *lp, float input);
float lowPassTS(LowPass_t *lp, float input);

#endif

