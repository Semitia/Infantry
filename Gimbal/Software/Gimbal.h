#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "RemoteCtrl.h"

#define GIM_RC_USART &usart3_if

/**
 * @brief 云台结构体
*/
typedef struct __Gimbal_t {

    RC_t rc;                   //遥控器结构体


}Gimbal_t;

void gimbalInit(Gimbal_t *gimbal);
void gimUpdate(Gimbal_t *gimbal);

#endif

