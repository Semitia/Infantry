#ifndef _COUNTER_H
#define _COUNTER_H

#include "sys.h"

/* Params START */
#define UINT_MAX UINT16_MAX  //定时器位数 tim2 4是32位，其他16位
#define COUNTER_APBx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define COUNTER_RCC_APBx_TIMx RCC_APB1Periph_TIM3
#define COUNTER_TIMx TIM3
#define COUNTER_DBGMCU_TIMx_STOP DBGMCU_TIM3_STOP
/* TIM_TimeBaseInitStructure配置到源码处修改 */
#define COUNTER_SAMPLING 1000000
/* Params END */

typedef struct SysTime_t
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} SysTime_t;

void COUNTER_Configuration(void);
#if UINT_MAX==UINT32_MAX
float GetDeltaT(uint32_t *cnt_last);
#elif UINT_MAX==UINT16_MAX
float GetDeltaT(uint16_t *cnt_last);
#endif
double GetDeltaT64(uint32_t *cnt_last);
void SysTimeUpdate(void);
float GetTime_s(void);
float GetTime_ms(void);
uint64_t GetTime_us(void);
void Delay(uint32_t Delay);
float GetDeltaTtoNow(uint32_t *cnt_last);

#endif // !_COUNTER_H
