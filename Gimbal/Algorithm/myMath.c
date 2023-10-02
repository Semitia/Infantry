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

