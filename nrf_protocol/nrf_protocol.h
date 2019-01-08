#ifndef NRF_PROTOCOL_H
#define NRF_PROTOCOL_H
#include "motor.h"
#include <stdint.h>


typedef struct
{
    uint16_t lift;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;
}ActuatorDOF;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t yaw;
}MotionDOF;

#define STATUS_DATA 0xA0
#define PID_DATA 0xA1


void Command_dispatch(u8* rx_buff);
void Command_patch(u8* tx_buff, u8 command_type,MOTION_STATUS* Motion_Status);



#endif
