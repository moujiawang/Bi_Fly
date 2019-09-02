#ifndef __INCPID_H_
#define __INCPID_H_

#include "stm32f10x.h"
#include "nrf_protocol.h"




//void IncPID_Init(PID_PARAS *PID_paras);
float PID_Update(PID_PARA* pid, const float error);
uint16_t IncPID_Cal(PID_PARA *PID_para, IMUFusion *Attitude,YPR_ID YPR_id);

#endif

