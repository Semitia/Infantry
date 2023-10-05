#include "GimbalTask.h"

Gimbal_t gimbal;
void gimbalTask(void *pvParameters) {
    uint32_t Gimbal_high_water;
    portTickType xLastWakeTime;
	const portTickType xFrequency = 2;

    gimbalInit(&gimbal);
    vTaskDelay(200);
    while(1)
    {
        xLastWakeTime = xTaskGetTickCount();
				gimUpdate(&gimbal);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#if INCLUDE_uxTaskGetStackHighWaterMark
		Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

