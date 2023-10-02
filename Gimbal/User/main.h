#ifndef __MAIN_H
#define __MAIN_H

#define SingleGyro 0 //�Ƿ�ʹ�õ���������
#define Robot_ID   46 // ��ͬ�������ò�ͬ�����
//  44     �¹���4�ų�
//  45     �¹���3�ų�
//  46     ����
//  47     �����¹���
/*Library*/
#include <stm32f4xx.h>	 
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

/*Mylib*/
#include "gpio.h"
#include "can1.h"
#include "can2.h"
#include "tim4.h"
#include "sys.h"
//#include "usart1.h"
#include "usart3.h"
#include "usart4_gryo.h"
#include "usart1.h"
#include "MicroSw.h"
#include "SteeringEngine.h"
#include "frictionwheel.h"
#include "iwdg.h"
#include "laser.h"
#include "pc_uart.h"
#include "bsp_spi_sdcard.h"


/*Algorithm*/
#include "pid.h"
#include "algorithmOfCRC.h"
#include "TD.h"
#include "queueData.h"

/*Task*/
#include "ZeroCheckTask.h"
//#include "DataSendTask.h"
//#include "DataReceivetask.h"
//#include "ActionTask.h"
//#include "GimbalTask.h"
#include "ShootTask.h"
//#include "ChassisTask.h"
#include "Start_Task.h"
#include "ControlTask.h"
#include "SDCardTask.h"

#include "RtosTaskCheck.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "mySensors.h"
#include "counter.h"
#include "tools.h"
#include "os_tick.h"
#include "debug.h"
#include "diskio.h"
#include "ff.h"

#define POWER_OFF 0
#define CHARGE_ENABLE 1

//IO�ڵ�ַӳ�� �ʺ�F405
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 


/*������ʼ�����ṹ��*/
typedef struct
{
	  int8_t gyro_pn;
	  int8_t motor_pn;
	  float FricMotor_pn[2];
	  float BodanMotor_pn;
		unsigned short MagOpen;
		unsigned short MagClose;
	  unsigned short Pitch_init;
	  unsigned short Yaw_init;
		unsigned short Solo_Yaw_init;	//����ģʽ��Yaw_init
		unsigned short Low_FrictionSpeed;
		unsigned short Medium_FrictionSpeed;
		unsigned short High_FrictionSpeed;
	  unsigned short PitchMotorID;
	  unsigned short YawMotorID;
	  unsigned short FricMotorID[2];
	  unsigned short BodanMotorID;
        short pitch_max_motor;
		short pitch_min_motor;
		short pitch_max_gyro;
		short pitch_min_gyro;
        short init_delta_pitch; //ƽ���������Ǻ͵���ǲ�ֵ
		
}RobotInit_Struct;

void BSP_Init(void);
void Robot_Init(void);

void Infantry_Init(void);
void delay_ms(unsigned long t);
void delay_us(unsigned long t);

void Offline_Check_task(void *pvParameters);
void Cmera_rising_edge(int *state,int cur_num);



#endif
