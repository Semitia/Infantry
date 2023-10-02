#ifndef __MAIN_H
#define __MAIN_H

#define SingleGyro 0 //是否使用单板陀螺仪
#define Robot_ID   46 // 不同机器人用不同的序号
//  44     下供弹4号车
//  45     下供弹3号车
//  46     舵轮
//  47     备用下供弹
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

//IO口地址映射 适合F405
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 


/*步兵初始参数结构体*/
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
		unsigned short Solo_Yaw_init;	//左单挑模式的Yaw_init
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
        short init_delta_pitch; //平地上陀螺仪和电机角差值
		
}RobotInit_Struct;

void BSP_Init(void);
void Robot_Init(void);

void Infantry_Init(void);
void delay_ms(unsigned long t);
void delay_us(unsigned long t);

void Offline_Check_task(void *pvParameters);
void Cmera_rising_edge(int *state,int cur_num);



#endif
