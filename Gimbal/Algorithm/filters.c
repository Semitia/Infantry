/**********************************************************************************************************
 * @文件     filters.c
 * @说明     常用滤波器封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-16
 **********************************************************************************************************/
#include "filters.h"

/**************************一阶低通滤波器**************************/
/**
 * @brief 一阶低通滤波器初始化（直接使用滤波系数）
 * @param lp 滤波器结构体
 * @param K  滤波系数
 * @retval None
 */
void lowPassInit(LowPass_t *lp, float K) {
    lp->K = K;
    lp->last = 0;
    return;
}

/**
 * @brief 一阶低通滤波器初始化（使用时间常数计算）
 * @param lp 滤波器结构体
 * @param tf 时间常数
 * @retval None
*/
void lowPassInitTS(LowPass_t *lp, float tf) {
    lp->tf = tf;
    lp->last = 0;
    lp->ts = xTaskGetTickCount();
    return;
}

/**
 * @brief 一阶低通滤波器
 * @param lp 滤波器结构体
 * @param input 输入值
 * @retval 滤波后的值
*/
float lowPass(LowPass_t *lp, float input) {
    lp->last = lp->last + lp->K * (input - lp->last);
    return lp->last;
}

float lowPassTS(LowPass_t *lp, float input) {
    float alpha = 0;
		TickType_t new_ts = xTaskGetTickCount();
    float dt = ( new_ts - lp->ts)/configTICK_RATE_HZ;
    if(dt>0.3f) {
        lp->last = input;
        lp->ts = new_ts;
        return input;
    } 
    alpha = lp->tf / (lp->tf + dt);
    lp->last = alpha*lp->last + (1.0f-alpha)*input;
    lp->ts = new_ts;
    return lp->last;
}

/**************************卡尔曼滤波器**************************/

// void updateKF(KalmanFilter_t *kf, Matrix_t z) {
//     kf->dt = (xTaskGetTickCount() - kf->t)/configTICK_RATE_HZ;
//     kf->t = xTaskGetTickCount();
//     kf->x = mul_matrix(kf->F, kf->x);
//     kf->P = add_matrix(mul_matrix(mul_matrix(kf->F, kf->P), tran_matrix(kf->F)), kf->Q);
//     kf->K = mul_matrix(mul_matrix(kf->P, tran_matrix(kf->H)), inv_matrix(add_matrix(mul_matrix(mul_matrix(kf->H, kf->P), tran_matrix(kf->H)), kf->R)));
//     kf->x = add_matrix(kf->x, mul_matrix(kf->K, sub_matrix(z, mul_matrix(kf->H, kf->x))));
//     kf->P = mul_matrix(sub_matrix(get_I(2), mul_matrix(kf->K, kf->H)), kf->P);
// }

