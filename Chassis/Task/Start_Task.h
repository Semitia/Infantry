#ifndef START_TASK_H
#define START_TASK_H

#include "sys.h"
#include "ChassisTask.h"

enum TASK_LIST
{
    CPU_TASK,
    POWERCONTROL_TASK,
    JUDGERECEIVE_TASK,
    GRAPHIC_TASK,
    OFFLINE_TASK,
    SDCARD_TASK,
    CHASSIS_TASK,
    TASK_NUM,
};

void CPU_task(void *pvParameters);
void startTast(void);

#endif
