#include "Gimbal.h"

/**
 * @brief 云台初始化
 * @param gimbal 云台结构体
*/
void gimbalInit(Gimbal_t *gimbal) {    
  remoteCtrlInit(&gimbal->rc);
  initINS(&gimbal->ins);
  motorInit(&gimbal->pose.moto_yaw, RM6020, YAW_ID);
  motorInit(&gimbal->pose.moto_pitch, RM6020, PITCH_ID);
  gimbal->pose.theta0 = THETA0;

  gimbal->pose.pitch_pos_pid.P = 7.0f;
  gimbal->pose.pitch_pos_pid.I = 0.001f;
  gimbal->pose.pitch_pos_pid.D = 0.0f;
  gimbal->pose.pitch_pos_pid.IMax = 40.0f;
  gimbal->pose.pitch_pos_pid.I_U = 0.4f;
  gimbal->pose.pitch_pos_pid.I_L = 0.2f;
  gimbal->pose.pitch_pos_pid.OutMax = 5.5f;
  gimbal->pose.pitch_pos_pid.RC_DF = 0.5f;

  gimbal->pose.pitch_spd_pid.P = 350.0f;
  gimbal->pose.pitch_spd_pid.I = 1.000f;
  gimbal->pose.pitch_spd_pid.D = 0.0f;
  gimbal->pose.pitch_spd_pid.IMax = 1000.0f;
  gimbal->pose.pitch_spd_pid.I_U = 60.0f;
  gimbal->pose.pitch_spd_pid.I_L = 20.0f;
  gimbal->pose.pitch_spd_pid.OutMax = 30000.0f;
  gimbal->pose.pitch_spd_pid.RC_DF = 0.5f;

  gimbal->pose.yaw_pos_pid.P = 27.0f;
  gimbal->pose.yaw_pos_pid.I = 0.000f;
  gimbal->pose.yaw_pos_pid.D = 1.2f;
  gimbal->pose.yaw_pos_pid.IMax = 30.0f;
  gimbal->pose.yaw_pos_pid.I_U = 20.0f;
  gimbal->pose.yaw_pos_pid.I_L = 10.0f;
  gimbal->pose.yaw_pos_pid.OutMax = 300.0f;
  gimbal->pose.yaw_pos_pid.RC_DF = 0.5f;
  
  gimbal->pose.yaw_spd_pid.P = 620.0f;
  gimbal->pose.yaw_spd_pid.I = 15.000f;
  gimbal->pose.yaw_spd_pid.D = 0.0f;
  gimbal->pose.yaw_spd_pid.IMax = 8000.0f;
  gimbal->pose.yaw_spd_pid.I_U = 50.0f;
  gimbal->pose.yaw_spd_pid.I_L = 20.0f;
  gimbal->pose.yaw_spd_pid.OutMax = 30000.0f;
  gimbal->pose.yaw_spd_pid.RC_DF = 0.5f;

	return;
}

/**
 * @brief 更新云台电机位姿
 * @param pose 云台位姿结构体
*/
void posUpdate(GimPosture_t *pose) {
	uint8_t pitch_recv=0, yaw_recv=0;
  CanRxMsg msg;
  CanRing_t *p;
  /* PITCH */
  p = PITCH_CAN_RX;
  //NVIC_DisableIRQ
  while(canRingHasMsg(p)) {
    CanRxMsg temp_msg = popCanMsg(p);
    if(temp_msg.StdId == pose->moto_pitch.id) { 
			msg = temp_msg;
			pitch_recv=1;
		}
    //应该还需要将误pop的消息重新压入队列

  }
  if(pitch_recv) motorUpdateAll(&pose->moto_pitch, msg.Data);
	
  /* YAW */  
  p = YAW_CAN_RX;
  while(canRingHasMsg(p)) {
    CanRxMsg temp_msg = popCanMsg(p);
    if(temp_msg.StdId == pose->moto_yaw.id) { 
			msg = temp_msg;
			yaw_recv=1;
		}
    //应该还需要将误pop的消息重新压入队列

  }
  if(yaw_recv) motorUpdateAll(&pose->moto_yaw, msg.Data);
  pose->theta = DEG2R(pose->moto_yaw.angle) - pose->theta0;
  return;
}

/**
 * @brief 设置云台位姿pitch，yaw电机电流
*/
void setPosCur(Gimbal_t *gimbal) {
  sendSingleMoto(&gimbal->pose.moto_yaw, YAW_CAN_TX);
  //sendSingleMoto(&gimbal->pose.moto_pitch, PITCH_CAN_TX);
  return;
}

/**
 * @brief 发送底盘状态
 * @param info 底盘交互信息结构体
*/
void sendChassisState(ChassisInfo_t *info) {
  return;
}


/**
 * @brief 发送底盘目标速度
 * @param info 底盘交互信息结构体
*/
void sendChassisVel(ChassisInfo_t *info) {
  short raw[3];
  CanTxMsg msg;
	msg.IDE = CAN_ID_STD;
	msg.RTR = CAN_RTR_DATA;
	msg.DLC = 0x08;
	msg.StdId = 0x101;

  raw[0] = info->tar_vel.x * 1000;
  raw[1] = info->tar_vel.y * 1000;
  raw[2] = info->tar_vel.w * 1000;

  msg.Data[0] = (raw[0]>>8)&0xff;
  msg.Data[1] = raw[0]&0xff;
  msg.Data[2] = (raw[1]>>8)&0xff;
  msg.Data[3] = raw[1]&0xff;
  msg.Data[4] = (raw[2]>>8)&0xff;
  msg.Data[5] = raw[2]&0xff;

  CAN_Transmit(CHASSIS_CANTX, &msg);
  return;
}

/**
 * @brief 更新云台数据
 * @param gimbal 云台结构体
*/
void gimUpdate(Gimbal_t *gimbal) {
  /* 更新遥控器指令 */
  if(gimbal->rc.usart_if->rx_flag) {
    updateRC(&gimbal->rc);                //更新数据并清除标志位
    gimbal->vel.x = linerMap(gimbal->rc.stick.ch0, MIN_STK_SIG, MAX_STK_SIG, MIN_SPEED, MAX_SPEED);
    gimbal->vel.y = linerMap(gimbal->rc.stick.ch1, MIN_STK_SIG, MAX_STK_SIG, MIN_SPEED, MAX_SPEED);
    if(gimbal->rc.stick.s1 == 3) {        //速度模式//右边中间
      //跟随
			//gimbal->pose.moto_yaw.target_speed = linerMap(gimbal->rc.stick.ch2, MIN_STK_SIG, MAX_STK_SIG, MAX_YAW_SPEED, MIN_YAW_SPEED);//摇杆似乎是反的，所以这里也反一下
      //阶跃
			if(gimbal->rc.stick.ch2 > 1200) gimbal->pose.moto_yaw.target_speed = 150;
			else if(gimbal->rc.stick.ch2 < 800) gimbal->pose.moto_yaw.target_speed = -150;
			else gimbal->pose.moto_yaw.target_speed = 0;
			
			//gimbal->pose.moto_pitch.target_speed = linerMap(gimbal->rc.stick.ch3, MIN_STK_SIG, MAX_STK_SIG, MIN_PITCH_SPEED, MAX_PITCH_SPEED);
    }
    else if(gimbal->rc.stick.s1 == 2) {   //位置模式//右边下面
      gimbal->pose.tar_yaw = linerMap(gimbal->rc.stick.ch2, MIN_STK_SIG, MAX_STK_SIG, MAX_YAW, MIN_YAW);
    }
  }
	gimbal->pose.tar_pitch = linerMap(gimbal->rc.stick.ch3, MIN_STK_SIG, MAX_STK_SIG, MIN_PITCH, MAX_PITCH);

  /* 更新云台电机位姿 */
  posUpdate(&gimbal->pose);

  /* 云台坐标系转换到底盘坐标系 */
  gimbal->chassis.tar_vel.x = gimbal->vel.x * cos(gimbal->pose.theta) + gimbal->vel.y * sin(gimbal->pose.theta);
  gimbal->chassis.tar_vel.y = gimbal->vel.y * cos(gimbal->pose.theta) - gimbal->vel.x * sin(gimbal->pose.theta);

  return;
}

/**
 * @brief 设置云台位姿
 * @note  依靠陀螺仪对pitch，yaw电机进行控制
 * @param gimbal 云台结构体
 * @return None
*/
void gimSetPose(Gimbal_t *gimbal) {
  //位置环
  //gimbal->pose.pitch_pos_pid.ActualValue = gimbal->ins.Pitch;
  //gimbal->pose.pitch_pos_pid.SetPoint = gimbal->pose.tar_pitch;
  //gimbal->pose.moto_pitch.target_speed = PID_Calc(&gimbal->pose.pitch_pos_pid);
  //速度环
  gimbal->pose.pitch_spd_pid.ActualValue = gimbal->ins.PitchSpeed;
  gimbal->pose.pitch_spd_pid.SetPoint = gimbal->pose.moto_pitch.target_speed;
  gimbal->pose.moto_pitch.target_current = - PID_Calc(&gimbal->pose.pitch_spd_pid);
  //位置环
  gimbal->pose.yaw_pos_pid.ActualValue = gimbal->ins.Yaw;
  gimbal->pose.yaw_pos_pid.SetPoint = gimbal->pose.tar_yaw;
  gimbal->pose.moto_yaw.target_speed = PID_Calc(&gimbal->pose.yaw_pos_pid);
  //速度环
  gimbal->pose.yaw_spd_pid.ActualValue = gimbal->ins.YawSpeed;
  gimbal->pose.yaw_spd_pid.SetPoint = gimbal->pose.moto_yaw.target_speed;
  gimbal->pose.moto_yaw.target_current = - PID_Calc(&gimbal->pose.yaw_spd_pid);
	
	
  if(gimbal->rc.stick.s1 == 1) return;      //急停
	if(gimbal->ins.Yaw > 70 || gimbal->ins.Yaw < -70) return;
	
  setPosCur(gimbal);
  return;
}

/**
 * @brief 电机测试
*/
void setMotorTest(void) {
	Motor_t moto;
  motorInit(&moto, RM6020, 0x207);
  moto.target_current = -5000;
  sendSingleMoto(&moto, PITCH_CAN_TX);
  return;
}




