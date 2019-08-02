#ifndef __TASK_H
#define __TASK_H

#include "nrf_protocol.h"

void Start_task(SYS_STATUS *SYS_status);

void Manual_task(SYS_STATUS *SYS_status);

void Flight_task(SYS_STATUS *SYS_status);

void Tuning_task(SYS_STATUS *SYS_status);

void Fault_task(SYS_STATUS *SYS_status);


void UpdateIMU_task(SYS_STATUS *SYS_status);

#endif
