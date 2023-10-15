#include "GimbalTask.h"

Gimbal_t gimbal;
/**
 * @brief 云台任务
 * @param[in] void
*/
void gimbalTask(void *pvParameters) {
    uint32_t Gimbal_high_water;
    portTickType xLastWakeTime;
	const portTickType xFrequency = 2;

    gimbalInit(&gimbal);
    vTaskDelay(5000);
    while(1)
    {
        xLastWakeTime = xTaskGetTickCount();
        gimUpdate(&gimbal);
        gimSetPose(&gimbal);
        sendChassisVel(&gimbal.chassis);
        //setMotorTest();
			
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#if INCLUDE_uxTaskGetStackHighWaterMark
		Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief 云台位姿估计任务
 * @param[in] void
 */
uint32_t GimbalEstimate_high_water;
void GimbalEstimate_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ
    vTaskDelay(5000);                   //等待惯导初始化完成
    while (1) {
        TaskCheck(INSTask);
        xLastWakeTime = xTaskGetTickCount();
        xSemaphoreTake(gimbal.ins.mutex, portMAX_DELAY);                //get mutex
        updateINS(&gimbal.ins);
        xSemaphoreGive(gimbal.ins.mutex);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#if INCLUDE_uxTaskGetStackHighWaterMark
	GimbalEstimate_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
