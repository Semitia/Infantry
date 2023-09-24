/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     底盘控制任务
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-24
 **********************************************************************************************************/
#include "ChassisTask.h"

Chassis_t chassis;
uint32_t Chassis_high_water = 0;
void Chassis_task(void *pvParameters)
{
	portTickType xLastWakeTime;   //config=0. 32v bits
	portTickType xFrequency = CHASSIS_CTRL_INTERVAL;
	chassisInit(&chassis, MECANUM_WHEEL);
	vTaskDelay(100);
	
	while (1) {
		xLastWakeTime = xTaskGetTickCount();
		
		// if(chassis.judge.recv_flag)//将它分为两个任务是因为这个数据接收太过耗时？
		// {
		// 	xTaskNotifyGive(User_Tasks[JUDGERECEIVE_TASK]);
		// }
		recvGimData(&chassis);
		moveCtrl(&chassis);
		sendGimData(&chassis);
		
		//VOFA_Send();
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 

	#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif
	}
}
