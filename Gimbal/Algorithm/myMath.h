#ifndef __MYMATH_H
#define __MYMATH_H

#include <math.h>
#include "Matrix.h"
#include "arm_math.h"

#define DEG2R(x) ((x)*PI /180.0f)                         //角度转弧度
#define R2DEG(x) ((x)*180.0f /PI)                         //弧度转角度
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
#define MAX(a,b) ((a)>(b)?(a):(b))                        //取较大值
#define MIN(a,b) ((a)<(b)?(a):(b))                        //取较小值

typedef struct __Roots_t{
    int isReal;     // 1 实数解，0 虚数解
    double Re[2];   // 实部
    double Im[2];   // 虚部
} Roots_t;

Roots_t solveQuadratic(double a, double b, double c);

#endif

