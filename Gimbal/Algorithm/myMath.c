#include "Mymath.h"

/**
 * @brief 一元二次方程求解
 * @param a 二次项系数
 * @param b 一次项系数
 * @param c 常数项
 * @return 方程的根
*/
Roots_t solveQuadratic(double a, double b, double c) {
    Roots_t roots;
    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) {
        roots.isReal = 0;
        roots.Re[0] = roots.Re[1] = -b / (2*a);
        roots.Im[0] = sqrt(-discriminant) / (2*a);
        roots.Im[1] = -sqrt(-discriminant) / (2*a);
    } else {
        roots.isReal = 1;
        roots.Im[0] = roots.Im[1] = 0.0;
        roots.Re[0] = (-b + sqrt(discriminant)) / (2*a);
        roots.Re[1] = (-b - sqrt(discriminant)) / (2*a);
    }
    return roots;
}

/**
 * @brief 线性映射函数
 * @param x 输入
 * @param in_min 输入最小值
 * @param in_max 输入最大值
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 * @return float 输出
*/
float linerMap(float x, float in_min, float in_max, float out_min, float out_max) {
    float out;
    out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return out;
}

