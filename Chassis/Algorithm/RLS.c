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
// void rlsUpdate(RLS_t *rls, MATRIX_TYPE *x_in, MATRIX_TYPE y) {
//     //print theta
//     printf("theta:\n");
//     M_print(rls->theta);

//     Matrix *x = Matrix_gen(rls->dim, 1, x_in);
//     MATRIX_TYPE e = y - M_mul(M_T(rls->theta),x)->data[0];         //注意data是一维数组

//     printf("error: %f\n", e);

//     /*计算增益矩阵*/
//     /*                  p(t-1) x(t)
//         K(t) = --------------------------------
//                 lambda + x(t)^T P(t-1) x(t)       */
//     Matrix *up = M_mul(rls->P, x);
//     Matrix *down = M_mul(M_mul(M_T(x), rls->P), x);
//     //Matrix *K = M_numul(M_mul(rls->P, x), 1/(M_mul(M_mul(M_T(x),rls->P), x)->data[0]+rls->lambda));
//     Matrix *K = M_numul(up, 1/(down->data[0]+rls->lambda));
//     /*更新参数*/
//     rls->theta = M_add_sub(1, rls->theta, 1, M_numul(K, e));
//     /*更新协方差矩阵*/
//     /*            1    --                            --
//         P(t) = --------| P(t-1) - K(t) x(t)^T P(t-1)  |
//                 lambda --                            --  */
//     rls->P = M_numul(M_add_sub(1, rls->P, 1, M_mul(M_mul(K, M_T(x)), rls->P)), 1/rls->lambda);
    
//     M_free(up);
//     M_free(down);
//     M_free(x);
//     M_free(K);
//     return;
// }
void rlsUpdate(RLS_t *rls, double *x_in, double y) {

    return;
}

