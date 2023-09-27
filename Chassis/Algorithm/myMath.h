#ifndef __MYMATH_H
#define __MYMATH_H

#include "arm_math.h"

#define DEG2R(x) ((x)*PI /180.0f)                         //角度转弧度
#define R2DEG(x) ((x)*180.0f /PI)                         //弧度转角度
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  //????

#endif

