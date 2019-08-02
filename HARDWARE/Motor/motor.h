#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "nrf_protocol.h"

#define End 1
#define FLY 0
#define CLIMB 1
#define STOP 2




void motor_init(void);
void Command_manage(int32_t Command_length[],MANUAL_STATUS* Manual_Status);
void Manual_command(const MANUAL_STATUS* Manual_Status);
void Flight_command(const FLIGHT_STATUS* Flight_Status);

void PID_command(SYS_STATUS *SYS_status);

#endif


