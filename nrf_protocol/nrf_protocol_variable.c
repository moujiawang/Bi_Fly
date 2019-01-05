#include "nrf_protocol.h"

static ActuatorDOF actuator_control;
static MotionDOF motion_control;

const ActuatorDOF* GetActuatorControl()
{
    return &actuator_control;
}

void SetActuatorControl(const ActuatorDOF* input)
{
    actuator_control = *input;
}

const MotionDOF* GetMotionControl()
{
    return &motion_control;
}

void SetMotionControl(const MotionDOF* input)
{
    motion_control = *input;
}