#include "nrf_protocol.h"
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

void command_dispatch(const uint8_t* rx_buff )
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
