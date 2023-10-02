#ifndef __REMOTECTRL_H
#define __REMOTECTRL_H

#include "usart3.h"

#define RC_RX_LEN 18

/**
 * @brief 摇杆结构体
*/
typedef struct __Stick_t{
    unsigned short ch0;
    unsigned short ch1;
    unsigned short ch2;
    unsigned short ch3;
    unsigned short s1;
    unsigned short s2;
}Stick_t;

/**
 * @brief 鼠标结构体
*/
typedef struct __Mouse_t
{
    short x;
    short y;
    short z;
    unsigned char press_l;
    unsigned char press_r;
}Mouse_t;

/**
 * @brief 键盘结构体
*/
typedef struct __Key_t
{
    unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key_t;

/**
 * @brief 遥控器键鼠接收结构体
*/
typedef struct __RC_t{
    Stick_t stick;
    Mouse_t mouse;                      
    Key_t key;                          
    char RCrecvd;                       //数据接收标志位
    char RCDisconnectCnt;               //遥控器掉线计数

    UsartIF_t *usart_if;                //串口接收结构体指针
}RC_t;

void remoteCtrlInit(RC_t *rc, UsartIF_t *usart_if);
void processRemoteData(RC_t *rc);

#endif


