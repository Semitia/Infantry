#ifndef __JUDGEMSG_H
#define __JUDGEMSG_H

#include "usart.h"

#define JUDGE_RXBUF_LEN 45
#define JUDGE_TXBUF_LEN 128

typedef struct __JudgeRecv_t {
    char HeatUpdate_NoUpdate;
    char SpeedUpdate_NoUpdate;

    //0x0201
    uint8_t robot_id;
    uint8_t RobotLevel;
    uint16_t remainHP;
    uint16_t maxHP;
    uint16_t HeatCool17;		//17mm枪口每秒冷却值
    uint16_t HeatMax17;			//17mm枪口热量上限
    uint16_t BulletSpeedMax17;	//17mm枪口上限速度
    uint16_t MaxPower;			//底盘功率限制上限

    //0x0202
    uint16_t realChassisOutV;
    uint16_t realChassisOutA;
    float realChassispower;
    uint16_t remainEnergy;       //剩余能量
    short shooterHeat17;

    //0x0207
    uint8_t bulletFreq;		//射击频率
    uint8_t ShootCpltFlag; //已射出一发子弹标志位

    //0x0208
    uint16_t num_17mm;
    uint16_t num_coin;

    //flag
    short HeatUpdateFlag;	

    //not in use
    uint8_t cardType;
    uint8_t CardIdx;

    float bulletSpeed;		//当前射速
    float LastbulletSpeed;

    uint8_t game_progress;

    uint16_t enemy_3_hp;
    uint16_t enemy_4_hp;
    uint16_t enemy_5_hp;

    uint16_t remain_time;
}JudgeRecv_t;

typedef struct __JudegSend_t {
		uint8_t data;
}JudgeSend_t;


/**
 * 裁判系统信息接收与处理结构体
*/
typedef struct __Judge_t {
    JudgeRecv_t recv_msg;           //接收消息
    UsartIF_t *usart_if;            //串口接口
    uint8_t *recv_flag;             //接收标志位，有新消息时为1
    uint8_t *send_flag;             //发送标志位，正在发送时为1

}Judge_t;

void judgeInit(Judge_t *judge, UsartIF_t *usart_if);
void recvJudgeMsg(Judge_t *judge);

#endif

