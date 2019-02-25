#ifndef __INCPID_H_
#define __INCPID_H_

#include "stm32f10x.h"


typedef struct
{
	int16_t Error_Int;		 	//e[k]			
	int16_t LastError_Int;		//e[k-1]
	int16_t PreError_Int;		//e[k-2]
	int16_t Error_Ext;			//e[k]
	int16_t PreError_Ext;		//e[k-1]
	int16_t LastError_Ext;		//e[k-2]
	char Kp_Int;					
	char Ki_Int;
	char Kd_Int;
	char Kp_Ext;
	char Ki_Ext;
	char Kd_Ext;
	int16_t PIDcal_ExtOut;
	int16_t PIDcal_IntOut;
	int16_t SetPoint_Ext;
	int16_t SetPoint_Int;
}PID_PARA;

void IncPID_Init(PID_PARA *PID_Yaw_para, PID_PARA *PID_Pitch_para, PID_PARA *PID_Roll_para);
uint16_t IncPID_Cal(PID_PARA *PID_para, float Angle_ActualVal, float rad_ActualVal);

#endif

