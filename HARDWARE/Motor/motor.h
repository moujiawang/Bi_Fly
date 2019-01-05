#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#define End 1
#define FLY 0
#define CLIMB 1
#define STOP 2

#define ROLL 0
#define PITCH 1
#define YAW 2

typedef struct
{ 
	uint16_t Roll_Pulse;
	uint16_t Pitch_Pulse;
	uint16_t Yaw_Pulse;
	uint16_t Fly_Pulse;
	uint16_t Climb_Pulse;
	uint16_t Control_Status;
	uint16_t Motion_Status;
}MOTOR_STATUS;

void motor_init(void);
void Command_manage(int32_t Command_length[]);

#endif
