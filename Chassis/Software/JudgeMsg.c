/**********************************************************************************************************
 * @文件     JudgeMsg.c
 * @说明     裁判系统通信封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-24
 *********************************************************************************************************/

#include "JudgeMsg.h"

/**
 * @brief  Judge初始化
*/
void judgeInit(Judge_t *judge, UsartIF_t *usart_if) {
    usartIfInit(usart_if, JUDGE_RXBUF_LEN, JUDGE_TXBUF_LEN);
    judge->usart_if = usart_if;
    judge->recv_flag = usart_if->rx_flag;
    return;
}

/**
 * @brief 裁判系统接收函数
 * @param judge 裁判系统结构体
*/
void recvJudgeMsg(Judge_t *judge) {
    uint8_t *p = judge->usart_if->rx_buf;
    
    
    return;
}

// float Last_chassisPower=0;
// char TickCount=0;
// uint16_t receivePower;
// u8 jdugetemp;
// u8 is_game_start;
// extern F105_Typedef F105;
// extern Power_Limit_type powerlimit;
// void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen)
// {
// 	uint16_t cmd_id;
// 	short PackPoint;
// 	memcpy(&SaveBuffer[JudgeBufBiggestSize],&ReceiveBuffer[0],JudgeBufBiggestSize);		//把ReceiveBuffer[0]地址拷贝到SaveBuffer[24], 依次拷贝24个, 把这一次接收的存到数组后方
// 	for(PackPoint=0;PackPoint<JudgeBufBiggestSize;PackPoint++)		//先处理前半段数据(在上一周期已接收完成)
// 	{
// 		if(SaveBuffer[PackPoint]==0xA5) 
// 		{	
// 			if((Verify_CRC8_Check_Sum(&SaveBuffer[PackPoint],5)==1))		//frame_header 五位数据校验，数组的最后一个应当为校验值
// 			{

// 				cmd_id=(SaveBuffer[PackPoint+6])&0xff;
// 				cmd_id=(cmd_id<<8)|SaveBuffer[PackPoint+5];  
// 				DataLen=SaveBuffer[PackPoint+2]&0xff;
// 				DataLen=(DataLen<<8)|SaveBuffer[PackPoint+1];
				
// 				//机器人状态数据
// 				if((cmd_id==0x0201)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9))) 
// 				{
// 					memcpy(&JudgeReceive.robot_id,&SaveBuffer[PackPoint+7+0],1);
// 					memcpy(&JudgeReceive.RobotLevel,&SaveBuffer[PackPoint+7+1],1);
// 					memcpy(&JudgeReceive.remainHP,&SaveBuffer[PackPoint+7+2],2);
// 					memcpy(&JudgeReceive.maxHP,&SaveBuffer[PackPoint+7+4],2);
// 					memcpy(&JudgeReceive.HeatCool17,&SaveBuffer[PackPoint+7+6],2);
// 					memcpy(&JudgeReceive.HeatMax17,&SaveBuffer[PackPoint+7+8],2);
// 					memcpy(&JudgeReceive.BulletSpeedMax17,&SaveBuffer[PackPoint+7+10],2);
// 					memcpy(&JudgeReceive.MaxPower,&SaveBuffer[PackPoint+7+24],2);
// 					if(JudgeReceive.MaxPower == 0)
// 						JudgeReceive.MaxPower = 60 ;
// 					//JudgeReceive.MaxPower = 80;
// 				}
                
//         if ((cmd_id == 0x0001) && (Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint], DataLen + 9))) //对应着5+4+4
// 				{
// 					JudgeReceive.game_progress = (SaveBuffer[PackPoint + 7 + 0] & 0xF0) >> 4; //取出高四位
//                     memcpy(&JudgeReceive.remain_time,&SaveBuffer[PackPoint + 7 + 1],2);
// //                    JudgeReceive.remain_time = SaveBuffer[PackPoint + 7 + 1];
//                     is_game_start = (JudgeReceive.game_progress >=0x04)?1:0;
// 				}
// 				if ((cmd_id == 0x0003) && (Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint], DataLen + 9)))
// 				{
//                    if(F105.Sendmessage.RobotRed == 0)
//                    {
//                    memcpy(&JudgeReceive.enemy_3_hp,&SaveBuffer[PackPoint+7+4],2);
//                    memcpy(&JudgeReceive.enemy_4_hp,&SaveBuffer[PackPoint+7+6],2);
//                    memcpy(&JudgeReceive.enemy_5_hp,&SaveBuffer[PackPoint+7+8],2);
// //				   JudgeReceive.enemy_3_hp = SaveBuffer[PackPoint + 7 + 4] << 8 | SaveBuffer[PackPoint + 7 + 5];
// //                 JudgeReceive.enemy_4_hp = SaveBuffer[PackPoint + 7 + 6] << 8 | SaveBuffer[PackPoint + 7 + 7];
// //                 JudgeReceive.enemy_5_hp = SaveBuffer[PackPoint + 7 + 8] << 8 | SaveBuffer[PackPoint + 7 + 9];
//                    }
//                    else if(F105.Sendmessage.RobotRed == 1)
//                    {
//                    memcpy(&JudgeReceive.enemy_3_hp,&SaveBuffer[PackPoint+7+20],2);
//                    memcpy(&JudgeReceive.enemy_4_hp,&SaveBuffer[PackPoint+7+22],2);
//                    memcpy(&JudgeReceive.enemy_5_hp,&SaveBuffer[PackPoint+7+24],2);
// //                   JudgeReceive.enemy_3_hp = SaveBuffer[PackPoint + 7 + 20] << 8 | SaveBuffer[PackPoint + 7 + 21];
// //                   JudgeReceive.enemy_4_hp = SaveBuffer[PackPoint + 7 + 22] << 8 | SaveBuffer[PackPoint + 7 + 23];
// //                   JudgeReceive.enemy_5_hp = SaveBuffer[PackPoint + 7 + 24] << 8 | SaveBuffer[PackPoint + 7 + 25];
//                    }
//                    //判断平衡步兵
//                    if(is_game_start == 1 && JudgeReceive.remain_time >= 415 && JudgeReceive.enemy_3_hp == 300)
//                        F105.which_balance = 3;
//                    if(is_game_start == 1 && JudgeReceive.remain_time >= 415 && JudgeReceive.enemy_4_hp == 300)
//                        F105.which_balance = 4;
//                    if(is_game_start == 1 && JudgeReceive.remain_time >= 415 && JudgeReceive.enemy_5_hp == 300)
//                        F105.which_balance = 5;
// 				}
					
// 				//实时功率、热量数据
// 				if((cmd_id==0x0202)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
// 				{
// 					memcpy(&JudgeReceive.realChassisOutV,&SaveBuffer[PackPoint+7+0],2);
// 					memcpy(&JudgeReceive.realChassisOutA,&SaveBuffer[PackPoint+7+2],2);
// 					memcpy(&JudgeReceive.realChassispower,&SaveBuffer[PackPoint+7+4],4);
// 					memcpy(&JudgeReceive.remainEnergy,&SaveBuffer[PackPoint+7+8],2);
// 					memcpy(&JudgeReceive.shooterHeat17,&SaveBuffer[PackPoint+7+10],2);                              // 2个字节
// 					Last_chassisPower=JudgeReceive.realChassispower;
					
// 					powerlimit.actual_i_2_ref = powerlimit.actual_i_2;
// 					powerlimit.actual_w_i_ref = powerlimit.actual_w_i;
// 					powerlimit.actual_referee_power = JudgeReceive.realChassispower;
// 					powerlimit.predict_power_10 = powerlimit.predict_power;
// //					powerlimit.predict_power_old_10 = powerlimit.predict_power_old;
// //					powerlimit.predict_power1_10 = powerlimit.predict_power1;
// //					powerlimit.predict_power5_10 = powerlimit.predict_power5;
// 				}
				
// 				//实时增益数据
// 				if((cmd_id==0x0204)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
// 				{
// 					Can2Send2(SaveBuffer[PackPoint+7+0]);
// 				}
// 				//
// 				if((cmd_id==0x206)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
// 				{
// 					jdugetemp = (SaveBuffer[PackPoint + 7] & 0xf0)>>4;
// 				}
				
				
				
// 				//实时射击信息
// 					if((cmd_id==0x0207)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
// 				{
// 					memcpy(&JudgeReceive.bulletFreq, &SaveBuffer[PackPoint+7+2],1);
// 					memcpy(&JudgeReceive.bulletSpeed,&SaveBuffer[PackPoint+7+3],4);
// 					JudgeReceive.ShootCpltFlag = 1;
// 				}
				
// 				//发弹量及金币
// 					if((cmd_id==0x0208)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
// 				{
// 					memcpy(&JudgeReceive.num_17mm, &SaveBuffer[PackPoint+7+0],2);
// 					memcpy(&JudgeReceive.num_coin,&SaveBuffer[PackPoint+7+4],2);
// 				}
				
// 				Can2Send1(&F105.Sendmessage);
				
// 			}
// 		}
// 	Robot_Disconnect.JudgeDisconnect =0;
// 	memcpy(&SaveBuffer[0],&SaveBuffer[JudgeBufBiggestSize],JudgeBufBiggestSize);		//把SaveBuffer[24]地址拷贝到SaveBuffer[0], 依次拷贝24个，把之前存到后面的数据提到前面，准备处理
// 	}
// }

