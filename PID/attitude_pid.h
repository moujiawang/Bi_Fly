#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H

#include "nrf_protocol.h"

void attitudeRatePID(PID_PARAS * pid_paras, IMUFusion * imu_data);	/* ���ٶȻ�PID */
void attitudeAnglePID(PID_PARAS * pid_paras, IMUFusion * imu_data);	/* �ǶȻ�PID */
void PID_command(SYS_STATUS *SYS_status);							/* PID���㣬������PID����*/
#endif

