/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿�������
 * @�汾  	 V1.0
 * @����     ����
 * @����     2023-9-24
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
		
		if(JudgeReveice_Flag)//������Ϊ������������Ϊ������ݽ���̫����ʱ��
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
