/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     底盘控制任务
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-24
 **********************************************************************************************************/
#include "ChassisTask.h"

Chassis_t chassis;
void Chassis_task(void *pvParameters)
{
	portTickType xLastWakeTime;   //config=0. 32v bits
	portTickType xFrequency = CHASSIS_CTRL_INTERVAL;
	chassisInit(&chassis, MECANUM_WHEEL);
	vTaskDelay(100);
	
	while (1) {
		xLastWakeTime = xTaskGetTickCount();
		
		if(JudgeReveice_Flag)//将它分为两个任务是因为这个数据接收太过耗时？
		{
		 xTaskNotifyGive(User_Tasks[JUDGERECEIVE_TASK]);
		}
		
		Chassis_Loop_Cal();
		BuildF105();
		Can2Send0(&F105);
		
		VOFA_Send();
	
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
	#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif
	}
}
