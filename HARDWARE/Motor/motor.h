#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "nrf_protocol.h"

#define End 1
#define FLY 0
#define CLIMB 1
#define STOP 2




void motor_init(MANUAL_STATUS *manual_status);
void Command_manage(int32_t Command_length[],MANUAL_STATUS* Manual_Status);
void Motor_action(const MANUAL_STATUS* Manual_status, const int16_t task_delay_num);
void Flight_command(const FLIGHT_STATUS* Flight_Status);

#endif


