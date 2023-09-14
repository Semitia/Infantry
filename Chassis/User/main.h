#ifndef __MAIN_H__
#define __MAIN_H__

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  //????

#define Robot_ID     46
// 44ÊÇ4ºÅ×ÔÊÊÓ¦
// 45ÊÇ3ºÅ×ÔÊÊÓ¦
// 46ÊÇ¶æÂÖ

#if Robot_ID == 46
#define Mecanum     2     //1Ê¹ÓÃÂó¿ËÄÉÄ·ÂÖ   0 Ê¹ÓÃÈ«ÏòÂÖ  2 Ê¹ÓÃ¶æÂÖ
#else
#define Mecanum     1     //1Ê¹ÓÃÂó¿ËÄÉÄ·ÂÖ   0 Ê¹ÓÃÈ«ÏòÂÖ  2 Ê¹ÓÃ¶æÂÖ
#endif

//Standard Lib
#include <stm32f4xx.h>	 
#include <stm32f4xx_conf.h>
#include "stm32f4xx_dac.h"
#include <string.h>
#include <stdint.h>
#include <arm_math.h>
#include <stdio.h>
#include <stdlib.h>

/*Hardware*/
#include "can1.h"
#include "can2.h"
#include "usart2.h"
#include "uart4.h"
#include "tim2.h"
#include "tim4.h"
#include "iwdg.h"
#include "adc.h"
#include "dac.h"
#include "ina260.h"
#include "i2c.h"
#include "counter.h"
#include "os_tick.h"
/*Algorithm*/
#include "pid.h"
#include "DataScope_DP.h"
#include "algorithmOfCRC.h"

/*Task*/
#include "DataReceiveTask.h"
// #include "DataSendTask.h"
// #include "ChassisTask.h"
// #include "PowerControlTask.h"
// #include "GraphicsSendTask.h"
// #include "CharSendTask.h"
 #include "StartTask.h"
// #include "JumpCal_Task.h"
// #include "SDCardTask.h"
// #include "ZeroCheckTask.h"

/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

#include "tools.h"


#define ABS(x) ((x)>0? (x):(-(x))) 
#define POWER_OFF 0
#define CHARGE_ENABLE 1

//IO¿ÚµØÖ·Ó³Éä ÊÊºÏF405
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40010814 
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40010C14 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40011014 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40011414 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40011814 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40011A14    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40011E14    

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40010810 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40010C10 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40011010 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40011410 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40011810 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40011A10 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40011E10

// ÊÊºÏF405
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //Êä³ö 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //ÊäÈë 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //Êä³ö 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //ÊäÈë 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //Êä³ö 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //ÊäÈë 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //Êä³ö 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //ÊäÈë 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //Êä³ö 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //ÊäÈë

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //Êä³ö 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //ÊäÈë

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //Êä³ö 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //ÊäÈë

typedef union
{
	float fdata;			//4?
	unsigned long idata;
}
FloatLongType;

typedef union
{
	float fdata;			//4?
	unsigned char idata[4];
}
FloatCharType;

typedef  struct{
short ChassisDisconnect[4];
short steerDisconnect[4];
short JudgeDisconnect;
short F405Disconnect;
short SuperPowerDisconnect;
float motor_rec_timer[8];
} roboDisconnect;
void BSP_Init(void);
void Robot_Init(void);
void Sys_Soft_Reset(void);
void System_Config(void);
void System_Init(void);
void Offline_Check_task(void *pvParameters);

#endif
