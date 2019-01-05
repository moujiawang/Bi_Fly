#ifndef __IMU_H
#define __IMU_H

#include "common.h"  //包含所有的驱动 头文件

#include <math.h>
#define M_PI  (float)3.1415926535
#define Upload_Speed  15   //data upload frequency Hz
#define upload_time (1000000/Upload_Speed)/2  // calculate upload time unit: us

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float ex_inte;
    float ey_inte;
    float ez_inte;
} OrientationEstimator;

typedef struct
{
	OrientationEstimator estimator;
	float ypr[3];
	uint32_t system_micrsecond;
	u8 upload_state;
	int16_t math_hz;
}IMUFusion;

//Mini IMU AHRS 解算的API
void imu_fusion_init(IMUFusion* imu_fusion);
void imu_fusion_do_run(IMUFusion* imu_fusion);
void IMU_init(OrientationEstimator* estimator); //初始化
void IMU_getYawPitchRoll(OrientationEstimator* estimator, float * ypr); //更新姿态
uint32_t micros(void);	//读取系统上电后的时间  单位 us 

#endif

//------------------End of File----------------------------
