#ifndef __MAIN_H__
#define __MAIN_H__

#include "sys.h"

#define Robot_ID     46
// 44ÊÇ4ºÅ×ÔÊÊÓ¦
// 45ÊÇ3ºÅ×ÔÊÊÓ¦
// 46ÊÇ¶æÂÖ

#if Robot_ID == 46
#define Mecanum     2     //1Ê¹ÓÃÂó¿ËÄÉÄ·ÂÖ   0 Ê¹ÓÃÈ«ÏòÂÖ  2 Ê¹ÓÃ¶æÂÖ
#else
#define Mecanum     1     //1Ê¹ÓÃÂó¿ËÄÉÄ·ÂÖ   0 Ê¹ÓÃÈ«ÏòÂÖ  2 Ê¹ÓÃ¶æÂÖ
#endif




/*SoftWare*/
// #include "Can.h"
// #include "Usart.h"

/*Hardware*/
#include "cap.h"
#include "usart2.h"
#include "uart4.h"
#include "tim2.h"
#include "tim.h"
#include "iwdg.h"
#include "adc.h"
#include "dac.h"
#include "ina260.h"
#include "i2c.h"
#include "counter.h"
//#include "os_tick.h"

/*Algorithm*/
//#include "DataScope_DP.h"
/*Task*/
//#include "DataReceiveTask.h"
// #include "DataSendTask.h"
//  #include "ChassisTask.h"
// #include "PowerControlTask.h"
// #include "GraphicsSendTask.h"
// #include "CharSendTask.h"
// #include "JumpCal_Task.h"
// #include "SDCardTask.h"
// #include "ZeroCheckTask.h"
#include "Start_Task.h"




#define POWER_OFF 0
#define CHARGE_ENABLE 1


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
