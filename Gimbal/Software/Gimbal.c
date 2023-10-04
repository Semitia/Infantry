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
  //initINS(&gimbal->ins);
  motorInit(&gimbal->moto_yaw, RM6020, YAW_ID);
  motorInit(&gimbal->moto_pitch, RM6020, PITCH_ID);

  gimbal->pitch_pos_pid.P = 0.2f;
  gimbal->pitch_pos_pid.I = 0.00001f;
  gimbal->pitch_pos_pid.D = 0.0f;
  gimbal->pitch_pos_pid.IMax = 40.0f;
  gimbal->pitch_pos_pid.I_U = 0.4f;
  gimbal->pitch_pos_pid.I_L = 0.2f;
  gimbal->pitch_pos_pid.OutMax = 5.5f;
  gimbal->pitch_pos_pid.RC_DF = 0.5f;

  gimbal->pitch_spd_pid.P = 0.2f;
  gimbal->pitch_spd_pid.I = 0.00001f;
  gimbal->pitch_spd_pid.D = 0.0f;
  gimbal->pitch_spd_pid.IMax = 40.0f;
  gimbal->pitch_spd_pid.I_U = 0.4f;
  gimbal->pitch_spd_pid.I_L = 0.2f;
  gimbal->pitch_spd_pid.OutMax = 5.5f;
  gimbal->pitch_spd_pid.RC_DF = 0.5f;

  gimbal->yaw_pos_pid.P = 0.2f;
  gimbal->yaw_pos_pid.I = 0.00001f;
  gimbal->yaw_pos_pid.D = 0.0f;
  gimbal->yaw_pos_pid.IMax = 40.0f;
  gimbal->yaw_pos_pid.I_U = 0.4f;
  gimbal->yaw_pos_pid.I_L = 0.2f;
  gimbal->yaw_pos_pid.OutMax = 5.5f;
  gimbal->yaw_pos_pid.RC_DF = 0.5f;
  
  gimbal->yaw_spd_pid.P = 0.2f;
  gimbal->yaw_spd_pid.I = 0.00001f;
  gimbal->yaw_spd_pid.D = 0.0f;
  gimbal->yaw_spd_pid.IMax = 40.0f;
  gimbal->yaw_spd_pid.I_U = 0.4f;
  gimbal->yaw_spd_pid.I_L = 0.2f;
  gimbal->yaw_spd_pid.OutMax = 5.5f;
  gimbal->yaw_spd_pid.RC_DF = 0.5f;



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
