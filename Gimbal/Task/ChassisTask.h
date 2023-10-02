#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"
typedef struct 
{
    short carSpeedx;
	short carSpeedy;
	short carSpeedw;
} ChassisSpeed_t;
enum POWERSTATE_Typedef
{
	BAT = 0,
	CAP,
	HalfCAP
};

void Chassis_Powerdown_Cal(void);
void Chassis_Act_Cal(Stick_t rc,Ket_t key);
void Chassis_SelfProtect_Cal(Stick_t rc,Ket_t key);
void Chassis_Solo_Cal(Stick_t rc,Ket_t key);
void Chassis_Jump_Cal(Stick_t rc,Ket_t key);

float ChassisPostionAngle_TranSform(short InitPos);

void Pid_ChassisPosition_Init(void);

void Chassis_CurrentPid_Cal(void);

void Current_Filter_Excu(void);
void Current_Set_Jump(void);

void Chassis_Flag_Set(void);
void Chassis_task(void *pvParameters);
#endif
