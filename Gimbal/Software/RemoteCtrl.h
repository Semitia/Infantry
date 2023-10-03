#ifndef __REMOTECTRL_H
#define __REMOTECTRL_H

#include "usart3.h"

#define RC_RX_LEN 18
#define RC_USART_IF &usart3_if
#define RC_USART_INIT USART3_Configuration

/**
 * @brief 摇杆结构体
*/
typedef struct __Stick_t{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t s1;
    uint16_t s2;
}Stick_t;

/**
 * @brief 鼠标结构体
*/
typedef struct __Mouse_t
{
    short x;
    short y;
    short z;
    uint8_t press_l;
    uint8_t press_r;
}Mouse_t;

/**
 * @brief 键盘结构体
*/
typedef struct __Key_t
{
    uint16_t w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key_t;

/**
 * @brief 遥控器键鼠接收结构体
*/
typedef struct __RC_t{
    Stick_t stick;
    Mouse_t mouse;                      
    Key_t key;                          
    uint8_t dis_cnt;                         //遥控器掉线计数

    UsartIF_t *usart_if;                  //串口接收结构体指针
}RC_t;

void remoteCtrlInit(RC_t *rc);
void updateRC(RC_t *rc);

#endif


