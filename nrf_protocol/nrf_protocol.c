#include "nrf_protocol.h"
#include "motor.h"
#include "write_protocol_variable.h"
#include <stdint.h>

void actuator_command(const uint8_t* data_address)
{
    ActuatorDOF temp_basic_control;
    temp_basic_control.lift  = *(uint16_t*)&data_address[0];
    temp_basic_control.yaw   = *( int16_t*)&data_address[2];
    temp_basic_control.pitch = *( int16_t*)&data_address[4];
    temp_basic_control.roll  = *( int16_t*)&data_address[6];
    SetActuatorControl(&temp_basic_control);
}

void motion_command(const uint8_t* data_address)
{
    MotionDOF temp_motion_control;
    temp_motion_control.x   = *(int16_t*)&data_address[0];
    temp_motion_control.y   = *(int16_t*)&data_address[2];
    temp_motion_control.z   = *(int16_t*)&data_address[4];
    temp_motion_control.yaw = *(int16_t*)&data_address[6];
    SetMotionControl(&temp_motion_control);
}

void pid_command(const uint8_t* data_address)
{

}

void Command_dispatch(u8* rx_buff )
{
    uint16_t function_code = *(uint16_t*)rx_buff;
    const uint8_t* data_addr = &rx_buff[2];

    if(0x00a0 == function_code)
    {
        actuator_command(data_addr);
    }
    else if(0x00a1 == function_code)
    {
        motion_command(data_addr);
    }
    else
    {
        //error
    }
}

void Command_patch(u8 tx_buff[], u8 command_type, MOTION_STATUS* Motion_Status)
{
	if(command_type == STATUS_DATA)
    {
        tx_buff[0] = STATUS_DATA;
        tx_buff[1] = (u8)Motion_Status->Fly_Pulse;
        tx_buff[2] = (u8)Motion_Status->Climb_Pulse;
        tx_buff[3] = (u8)Motion_Status->Roll_Pulse;
        tx_buff[4] = (u8)Motion_Status->Pitch_Pulse;
        tx_buff[5] = (u8)Motion_Status->Yaw_Pulse;
        tx_buff[6] = Motion_Status->test;
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
