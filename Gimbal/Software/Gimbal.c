#include "Gimbal.h"

void gimbalInit(Gimbal_t *gimbal) {    
  remoteCtrlInit(&gimbal->rc, GIM_RC_USART);
	return;
}

