#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "nrf_protocol.h"

#define End 1
#define FLY 0
#define CLIMB 1
#define STOP 2

#define ROLL 0
#define PITCH 1
#define YAW 2



void motor_init(void);
void Command_manage(int32_t Command_length[],ACTUATOR_STATUS* Actuator_Status);
void Actuator_command(const ACTUATOR_STATUS* Actuator_Status);
void Motion_command(const MOTION_STATUS* Motion_Status);
void PID_command(SYS_STATUS *SYS_status);

#endif


