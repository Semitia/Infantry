#include "Gimbal.h"

/**
 * @brief 将遥控器摇杆信号映射到速度
*/
float mapSignToSpd(uint16_t signal) {
    float speed = (float)(signal - MIN_STK_SIG) / (MAX_STK_SIG - MIN_STK_SIG) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
    return speed;
}

/**
 * @brief 云台初始化
 * @param gimbal 云台结构体
*/
void gimbalInit(Gimbal_t *gimbal) {    
  remoteCtrlInit(&gimbal->rc);
	return;
}

/**
 * @brief 更新云台数据
 * @param gimbal 云台结构体
*/
void gimUpdate(Gimbal_t *gimbal) {
  if(gimbal->rc.usart_if->rx_flag) {
    updateRC(&gimbal->rc);                //更新数据并清除标志位
    gimbal->vel.x = mapSignToSpd(gimbal->rc.stick.ch0);
    gimbal->vel.y = mapSignToSpd(gimbal->rc.stick.ch1);

    if(gimbal->rc.stick.s1 == 1) {        //紧急停止
      gimbal->vel.x = 0;
      gimbal->vel.y = 0;
    }
  }
  


  return;
}
