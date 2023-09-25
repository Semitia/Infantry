#ifndef __SYS_H
#define __SYS_H

#include "stm32f4xx.h"
// #include "stm32f4xx_conf.h"
#include "stm32f4xx_dac.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//User Lib
#include "myMath.h"

//Standard Lib
#include <string.h>
#include <stdint.h>
#include <arm_math.h>
#include <stdio.h>
#include <stdlib.h>



//IO口地址映射 适合F405
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

// 适合F405
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

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

void delay_us(unsigned long t);
void delay_us_f(float us);

#endif

