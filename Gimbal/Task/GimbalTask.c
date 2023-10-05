#include "GimbalTask.h"

void PitchCan2Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x1FF;
	tempX = LIMIT_MAX_MIN(tempX, 30000, -30000); // 30000


		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);

	
//	memset(&tx_message.Data,0,8);
	
	CAN_Transmit(CAN2, &tx_message);
}

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
				//setMotorTest();
				//PitchCan2Send(5000);
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
