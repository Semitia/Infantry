#include "RemoteCtrl.h"

/**
 * @brief 遥控器初始化
 * @param rc 遥控器结构体指针
 * @param usart_if 串口接收结构体指针
*/
void remoteCtrlInit(RC_t *rc, UsartIF_t *usart_if) {
    rc->usart_if = usart_if;
    rc->RCrecvd = 0;
    rc->RCDisconnectCnt = 0;
    usartIfInit(usart_if, RC_RX_LEN, 0);
}

void processRemoteData(RC_t *rc) {
    uint8_t *buf = rc->usart_if->rx_buf;
    rc->stick.ch0 = (buf[0] | (buf[1] << 8)) & 0x07ff;                                 // Channel 0
    rc->stick.ch1 = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff;                          // Channel 1
    rc->stick.ch2 = ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07ff;         // Channel 2
    rc->stick.ch3 = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff;                          // Channel 3
    rc->stick.s1 = ((buf[5] >> 4) & 0x0003);                                            // Switch left
    rc->stick.s2 = ((buf[5] >> 6) & 0x0003);
    rc->mouse.x = buf[6] | (buf[7] << 8);                                            // Mouse X axis
    rc->mouse.y = buf[8] | (buf[9] << 8);                                            // Mouse Y axis
    rc->mouse.z = buf[10] | (buf[11] << 8);                                          // Mouse Z axis
    rc->mouse.press_l = buf[12];                                                     // Mouse Left Is Press ?
    rc->mouse.press_r = buf[13];                                                     // Mouse Right Is Press ?
    rc->key.w = buf[14] & 0x01;                                                      // KeyBoard value
    rc->key.s = (buf[14] >> 1) & 0x01;
    rc->key.a = (buf[14] >> 2) & 0x01;
    rc->key.d = (buf[14] >> 3) & 0x01;
    rc->key.shift = (buf[14] >> 4) & 0x01;
    rc->key.ctrl = (buf[14] >> 5) & 0x01;
    rc->key.q = (buf[14] >> 6) & 0x01;
    rc->key.e = (buf[14] >> 7) & 0x01;
    rc->key.r = (buf[15]) & 0x01;
    rc->key.f = (buf[15] >> 1) & 0x01;
    rc->key.g = (buf[15] >> 2) & 0x01;
    rc->key.z = (buf[15] >> 3) & 0x01;
    rc->key.x = (buf[15] >> 4) & 0x01;
    rc->key.c = (buf[15] >> 5) & 0x01;
    rc->key.v = (buf[15] >> 6) & 0x01;
    rc->key.b = (buf[15] >> 7) & 0x01;
    rc->RCrecvd = 1;
    rc->RCDisconnectCnt = 0;
    if((rc->stick.ch0 - 1024 < 20) && (rc->stick.ch0 - 1024 > -20)) rc->stick.ch0 = 1024;
    if((rc->stick.ch1 - 1024 < 20) && (rc->stick.ch1 - 1024 > -20)) rc->stick.ch1 = 1024;
    if((rc->stick.ch2 - 1024 < 20) && (rc->stick.ch2 - 1024 > -20)) rc->stick.ch2 = 1024;
    if((rc->stick.ch3 - 1024 < 20) && (rc->stick.ch3 - 1024 > -20)) rc->stick.ch3 = 1024;
    return;
}
