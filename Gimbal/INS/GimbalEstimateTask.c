#include "GimbalEstimateTask.h"

INS_t ins;
/**
 * @brief 云台位姿估计任务
 * @param[in] void
 */
uint32_t GimbalEstimate_high_water;
void GimbalEstimate_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ
		initINS(&ins);
    vTaskDelay(50);

    while (1)
    {
      TaskCheck(INSTask);
        xLastWakeTime = xTaskGetTickCount();

        updateINS(&ins);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#if INCLUDE_uxTaskGetStackHighWaterMark
				GimbalEstimate_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
