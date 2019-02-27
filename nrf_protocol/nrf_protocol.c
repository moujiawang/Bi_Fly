#include "nrf_protocol.h"
#include "motor.h"
#include "write_protocol_variable.h"
#include <stdint.h>

void actuator_command(const uint8_t* data_address)
{
//    ACTUATOR_STATUS temp_basic_control;
//    temp_basic_control.lift  = *(uint16_t*)&data_address[0];
//    temp_basic_control.yaw   = *( int16_t*)&data_address[2];
//    temp_basic_control.pitch = *( int16_t*)&data_address[4];
//    temp_basic_control.roll  = *( int16_t*)&data_address[6];
//    SetActuatorControl(&temp_basic_control);
}

void motion_command(const uint8_t* data_address)
{
//    MOTION_STATUS temp_motion_control;
//    temp_motion_control.x   = *(int16_t*)&data_address[0];
//    temp_motion_control.y   = *(int16_t*)&data_address[2];
//    temp_motion_control.z   = *(int16_t*)&data_address[4];
//    temp_motion_control.orientation = *(int16_t*)&data_address[6];
//    SetMotionControl(&temp_motion_control);
}

void pid_command(const uint8_t* data_address)
{

}

u8 Command_dispatch(u8 *rx_buff, ACTUATOR_STATUS *Actuator_status, MOTION_STATUS *Motion_status, PID_PARAS *PID_paras)
{
	uint16_t Mode_ID = *rx_buff;
//	const uint8_t* data_addr = &rx_buff[1];
	switch(Mode_ID)
	{
		case 0xa0:actuator_command(data_addr);break;
		case 0xa1:motion_command(data_addr);break;
		case 0xa2:;break;
		
	}
	
	return Mode_ID;

}

void Command_patch(u8 tx_buff[], u8 command_type, ACTUATOR_STATUS* Actuator_Status)
{
	if(command_type == STATUS_DATA)
    {
        tx_buff[0] = STATUS_DATA;
        tx_buff[1] = (u8)Actuator_Status->Fly_Pulse;
        tx_buff[2] = (u8)Actuator_Status->Climb_Pulse;
        tx_buff[3] = (u8)Actuator_Status->Roll_Pulse;
        tx_buff[4] = (u8)Actuator_Status->Pitch_Pulse;
        tx_buff[5] = (u8)Actuator_Status->Yaw_Pulse;
        tx_buff[6] = Actuator_Status->test;
        tx_buff[7] = 0;
        tx_buff[8] = 0;
        tx_buff[9] = 0;
    }
    if(command_type == PID_DATA)
    {
        tx_buff[0] = PID_DATA;
        tx_buff[1] = 0;
        tx_buff[2] = 0;
        tx_buff[3] = 0;
        tx_buff[4] = 0;
        tx_buff[5] = 0;
        tx_buff[6] = 0;
        tx_buff[7] = 0;
        tx_buff[8] = 0;
        tx_buff[9] = 0;
    }
}
