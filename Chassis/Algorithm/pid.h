#ifndef __PID_H
#define __PID_H

#include "sys.h"

/**
 * @brief  PID结构体
*/
typedef struct __PID_t{
		float SetPoint;			//设定目标值
		float SetPointLast;
		float deadband; //死区
		
		float P;						//比例常数
		float I;						//积分常数
		float D;						//微分常数
		
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
		float dError;
		
		float ErrorMax;			//偏差上限 超出偏差则不计算积分作用
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	  float OutMax;       //限幅
}Pid_t;

float PID_Calc(Pid_t * P, float target, float ActualValue);
#endif
