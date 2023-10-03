#include "Gimbal.h"

void gimbalInit(Gimbal_t *gimbal) {    
  remoteCtrlInit(&gimbal->rc);
	return;
}

void gimUpdate(Gimbal_t *gimbal) {

  if(gimbal->rc.usart_if->rx_flag) {
    updateRC(&gimbal->rc);          //更新数据并清除标志位
  }

  return;
}
