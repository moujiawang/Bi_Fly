#ifndef NRF_PROTOCOL_H
#define NRF_PROTOCOL_H
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

#define STATUS_DATA 0x00A0;
#define PID_DATA 0x00A1;


void command_dispatch(const uint8_t* rx_buff );

#endif
