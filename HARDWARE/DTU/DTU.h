#ifndef __DTU_H_
#define __DTU_H_

#include "stm32f10x.h"

#define End 1
#define FLY 0
#define CLIMB 1
#define STOP 2

#define ROLL 0
#define PITCH 1
#define YAW 2

#define TIM4_PERIOD 8000

void DTU_init(void);
void Command_manage(int32_t Command_length[]);

#endif 
