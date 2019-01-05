#ifndef NRF_PROTOCOL_VARIABLE_H
#define NRF_PROTOCOL_VARIABLE_H
#include "nrf_protocol.h"

void SetActuatorControl(const ActuatorDOF* input);
void SetMotionControl(const MotionDOF* input);

#endif