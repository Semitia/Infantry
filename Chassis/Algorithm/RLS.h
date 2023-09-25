#ifndef __RLS_H
#define __RLS_H

#include "Matrix.h"

/**
 * @brief  Recursive Least Square
*/
typedef struct __RLS_t{
    int dim;            //维度
    Matrix_t theta;      //参数向量
    Matrix_t P;          //协方差矩阵
    float lambda;       //遗忘因子
}RLS_t;

void rlsInit(RLS_t *rls, int dim, double theta[][INV], double P[][INV], double lambda);
void rlsUpdate(RLS_t *rls, double *x_in, double y);

#endif

