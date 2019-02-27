#ifndef NRF_PROTOCOL_H
#define NRF_PROTOCOL_H

#include <stdint.h>
#include "stm32f10x.h"

typedef struct
{ 
	uint16_t Roll_Pulse;
	uint16_t Pitch_Pulse;
	uint16_t Yaw_Pulse;
	uint16_t Fly_Pulse;
	uint16_t Climb_Pulse;
	uint16_t Control_Status;
	uint16_t Fly_or_Climb_Status;
	u8 test;
}ACTUATOR_STATUS;

//typedef struct
//{
//    uint16_t lift;
//    int16_t  yaw;
//    int16_t  pitch;
//    int16_t  roll;
//}ACTUATOR_STATUS;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t orientation;
}MOTION_STATUS;

typedef struct
{
	int16_t Error_Int;		 			//e[k]			
	int16_t LastError_Int;			//e[k-1]
	int16_t PreError_Int;				//e[k-2]
	int16_t Error_Ext;					//e[k]
	int16_t PreError_Ext;				//e[k-1]
	int16_t LastError_Ext;			//e[k-2]
	uint8_t Kp_Int;					
	uint8_t Ki_Int;
	uint8_t Kd_Int;
	uint8_t Kp_Ext;
	uint8_t Ki_Ext;
	uint8_t Kd_Ext;
	int16_t PIDcal_ExtOut;
	int16_t PIDcal_IntOut;
	int16_t SetPoint_Ext;
	int16_t SetPoint_Int;
}PID_PARA;

typedef struct
{
	PID_PARA PID_Yaw_para;		
	PID_PARA PID_Pitch_para;
	PID_PARA PID_Roll_para;
}PID_PARAS;

#define STATUS_DATA 0xA0
#define PID_DATA 0xA1


u8 Command_dispatch(u8 *rx_buff, ACTUATOR_STATUS *Actuator_status, MOTION_STATUS *Motion_status, PID_PARAS *PID_paras);
void Command_patch(u8* tx_buff, u8 command_type,ACTUATOR_STATUS* Actuator_Status);



#endif
