#ifndef __KINEMATIC_H
#define __KINEMATIC_H

/**
 * 速度结构体
*/
typedef struct __Velocity_t {
    float x;  // x方向速度
    float y;  // y方向速度
    float w;    // 角速度
}Velocity_t;

// void mecanumForKinematic(Chassis_t *chassis);
// void mecanumInvKinematic(Chassis_t *chassis);
// void omniForKinematic(Chassis_t *chassis);
// void omniInvKinematic(Chassis_t *chassis);
// void steeringForKinematic(Chassis_t *chassis);
// void steeringInvKinematic(Chassis_t *chassis);

typedef struct __Mecanum_t {
    //车盘机械尺寸参数

} Mecanum_t;

void mecanumForKinematic(Mecanum_t *mecanum, float *wheel_speed);
void mecanumInvKinematic(Mecanum_t *mecanum, , float *wheel_speed);

#endif