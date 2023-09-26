#include "RLS.h"

/**
 * @brief  递推最小二乘法初始化
 * @param dim 维度
 * @param theta 参数向量
 * @param P 协方差矩阵
*/
void rlsInit(RLS_t *rls, int dim, double theta[][INV], double P[][INV], double lambda){
    rls->lambda = lambda;
    rls->dim = dim;
    rls->theta = matrix_init(dim, 1, theta);
    rls->P = matrix_init(dim, dim, P);
    return;
}

/**
 * @brief  根据新输入更新参数
 * @param x 新输入（数组）
 * @param y 新输出
*/
void rlsUpdate(RLS_t *rls, double *x_in, double y) {
    double temp[INV][INV] = {{x_in[0]}, {x_in[1]}, {x_in[2]}};//因为矩阵定义的限制，用中间变量过渡一下，换指针可以解决这个问题
    Matrix_t x = matrix_init(rls->dim, 1,temp);
    double e = y - mul_matrix(tran_matrix(rls->theta),x).matrix[0][0];         //注意data是一维数组
    /*******************计算增益矩阵******************/
    /*                  p(t-1) x(t)
        K(t) = --------------------------------
                lambda + x(t)^T P(t-1) x(t)         */
    Matrix_t K = numdiv_matrix(mul_matrix(rls->P,x),
                            rls->lambda + mul_matrix(mul_matrix(tran_matrix(x),rls->P), x).matrix[0][0]);
    
    /*******************更新参数**********************/
    rls->theta = add_matrix(rls->theta, numul_matrix(K, e));

    /*******************更新协方差矩阵****************/
    /*            1    --                            --
        P(t) = --------| P(t-1) - K(t) x(t)^T P(t-1)  |
                lambda --                            --  */
    rls->P = numdiv_matrix(sub_matrix(rls->P, mul_matrix(mul_matrix(K, tran_matrix(x)), rls->P)),
                        rls->lambda);
    return;
}

