#ifndef __TASK_H
#define __TASK_H

#include "nrf_protocol.h"

#define START_TASK_DELAY 100
#define MANUAL_TASK_DELAY 100
#define FLIGHT_TASK_DELAY 100
#define TUNING_TASK_DELAY 25
#define FAULT_TASK_DELAY 100
#define IMU_UPDATE_DALAY 4


void Start_task(SYS_STATUS *SYS_status);

void Manual_task(SYS_STATUS *SYS_status);

void Flight_task(SYS_STATUS *SYS_status);

void Tuning_task(SYS_STATUS *SYS_status);

void Fault_task(SYS_STATUS *SYS_status);

#endif
