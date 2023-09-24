#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__

#include "main.h"
#include "chassis.h"

#define Vol2Current(x,y) (((x) - (y)*76.0)/1.35f) //���루��ѹ��ת�٣��������Ӧ�ĵ���
#define CAP_MAX_W      7000
#define Rand_S         0.5f   //���ڳ���
#define RandThreshold  0.4f   //ֱ��ƫ��
#define RANDA          1.3f   //���ҷ�ֵ

void Chassis_task(void *pvParameters);

#endif
