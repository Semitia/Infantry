#include "Gimbal.h"

/**
 * @brief 云台初始化
 * @param gimbal 云台结构体
*/
void gimbalInit(Gimbal_t *gimbal) {    
  remoteCtrlInit(&gimbal->rc);
  //initINS(&gimbal->ins);
  motorInit(&gimbal->pose.moto_yaw, RM6020, YAW_ID);
  motorInit(&gimbal->pose.moto_pitch, RM6020, PITCH_ID);

  gimbal->pose.pitch_pos_pid.P = 0.2f;
  gimbal->pose.pitch_pos_pid.I = 0.00001f;
  gimbal->pose.pitch_pos_pid.D = 0.0f;
  gimbal->pose.pitch_pos_pid.IMax = 40.0f;
  gimbal->pose.pitch_pos_pid.I_U = 0.4f;
  gimbal->pose.pitch_pos_pid.I_L = 0.2f;
  gimbal->pose.pitch_pos_pid.OutMax = 5.5f;
  gimbal->pose.pitch_pos_pid.RC_DF = 0.5f;

  gimbal->pose.pitch_spd_pid.P = 0.2f;
  gimbal->pose.pitch_spd_pid.I = 0.00001f;
  gimbal->pose.pitch_spd_pid.D = 0.0f;
  gimbal->pose.pitch_spd_pid.IMax = 40.0f;
  gimbal->pose.pitch_spd_pid.I_U = 0.4f;
  gimbal->pose.pitch_spd_pid.I_L = 0.2f;
  gimbal->pose.pitch_spd_pid.OutMax = 5.5f;
  gimbal->pose.pitch_spd_pid.RC_DF = 0.5f;

  gimbal->pose.yaw_pos_pid.P = 0.2f;
  gimbal->pose.yaw_pos_pid.I = 0.00001f;
  gimbal->pose.yaw_pos_pid.D = 0.0f;
  gimbal->pose.yaw_pos_pid.IMax = 40.0f;
  gimbal->pose.yaw_pos_pid.I_U = 0.4f;
  gimbal->pose.yaw_pos_pid.I_L = 0.2f;
  gimbal->pose.yaw_pos_pid.OutMax = 5.5f;
  gimbal->pose.yaw_pos_pid.RC_DF = 0.5f;
  
  gimbal->pose.yaw_spd_pid.P = 0.2f;
  gimbal->pose.yaw_spd_pid.I = 0.00001f;
  gimbal->pose.yaw_spd_pid.D = 0.0f;
  gimbal->pose.yaw_spd_pid.IMax = 40.0f;
  gimbal->pose.yaw_spd_pid.I_U = 0.4f;
  gimbal->pose.yaw_spd_pid.I_L = 0.2f;
  gimbal->pose.yaw_spd_pid.OutMax = 5.5f;
  gimbal->pose.yaw_spd_pid.RC_DF = 0.5f;



	return;
}

/**
 * @brief 设置云台位姿pitch，yaw电机电流
*/
void setPosCur(Gimbal_t *gimbal) {
  sendSingleMoto(&gimbal->pose.moto_yaw, YAW_CAN_TX);
  sendSingleMoto(&gimbal->pose.moto_pitch, PITCH_CAN_TX);
  return;
}

/**
 * @brief 更新云台数据
 * @param gimbal 云台结构体
*/
void gimUpdate(Gimbal_t *gimbal) {
  if(gimbal->rc.usart_if->rx_flag) {
    updateRC(&gimbal->rc);                //更新数据并清除标志位
    gimbal->vel.x = linerMap(gimbal->rc.stick.ch0, MIN_STK_SIG, MAX_STK_SIG, MIN_SPEED, MAX_SPEED);
    gimbal->vel.y = linerMap(gimbal->rc.stick.ch1, MIN_STK_SIG, MAX_STK_SIG, MIN_SPEED, MAX_SPEED);
    gimbal->pose.tar_pitch = linerMap(gimbal->rc.stick.ch2, MIN_STK_SIG, MAX_STK_SIG, MIN_PITCH, MAX_PITCH);
    gimbal->pose.tar_yaw = linerMap(gimbal->rc.stick.ch3, MIN_STK_SIG, MAX_STK_SIG, MIN_YAW, MAX_YAW);
  
    if(gimbal->rc.stick.s1 == 1) {        //紧急停止
      gimbal->vel.x = 0;
      gimbal->vel.y = 0;
    }
  }
  
  
  return;
}

