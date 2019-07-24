#ifndef NRF_PROTOCOL_H
#define NRF_PROTOCOL_H

#include <stdint.h>
#include "stm32f10x.h"

typedef enum{ TX_MODE, RX_MODE } NRF24L01_MODE; 		//TX_MODE->0,RX_MODE->1
typedef enum{ PID_YAW, PID_PITCH,PID_ROLL } PID_ID;     //PID_YAW->0, PID_PITCH->1,PID_ROLL->2
typedef enum{ TX_WAIT, TX_GO } TX_FLAG;					//Tx_Flag
typedef enum{ YAW, PITCH, ROLL } YPR_ID;				//YAW->0, PITCH->1,ROLL->2
typedef enum{ 	
				FAULT_MODE = 0x00, 
				START_MODE = 0x08, 
				MANUAL_MODE = 0x10,
				FLIGHT_MODE = 0x18,
				TUNING_MODE = 0x38 
			} MODE_ID;									//FAULT_MODE = 0x00, START_MODE = 0x08, MANUAL_MODE = 0x10,FLIGHT_MODE = 0x18,TUNING_MODE = 0x38


typedef struct
{
    float q0;						//q0-q3四元数
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
	float ypr_rate[3];
	uint32_t system_micrsecond;
	u8 upload_state;
	int16_t math_hz;
}IMUFusion;

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
	int16_t Error_Int;		 		//e[k]			
	int16_t LastError_Int;			//e[k-1]
	int16_t PreError_Int;			//e[k-2]
	int16_t Error_Ext;				//e[k]
	int16_t PreError_Ext;			//e[k-1]
	int16_t LastError_Ext;			//e[k-2]
	u8 Kp_Int;					
	u8 Ki_Int;
	u8 Kd_Int;
	u8 Kp_Ext;
	u8 Ki_Ext;
	u8 Kd_Ext;
	int8_t PIDcal_ExtOut;
	int16_t PIDcal_IntOut;
	int16_t SetPoint_Int;
	int16_t SetPoint_Ext;
}PID_PARA;

typedef struct
{
	YPR_ID PID_id;
	PID_PARA PID_YPR_para[3];		
}PID_PARAS;

typedef struct
{
	u8 DTU_NRF_Status;
	IMUFusion imu_fusion_module;
	ACTUATOR_STATUS Actuator_Status;
	MOTION_STATUS Motion_Status;
	PID_PARAS PID_Paras;
}SYS_STATUS;


//NRF24L01 和 接收机是否在线标志号

#define NRF_ON            0x01					//NRF在线，表明飞行器端有可以与主控芯进行通讯的NRF模块
#define NRF_OFF           0xfe					//NRF不在线

//NRF24L01和接收机通讯标志号
#define NRF_CONNECTED     0x02					//NRF通讯成功
#define NRF_DISCONNECTED  0xFD					//NRF通讯失败
#define DTU_OK            0x03					//DTU接收机通讯正常就表明在线，DTU无是否在线的检测标志位



 

u8 Mode_init(SYS_STATUS *SYS_status);
u8 Command_dispatch(u8 *rx_buff, SYS_STATUS *SYS_status);
void Command_patch(u8 *tx_buff, SYS_STATUS *SYS_status, u8 mode_id);
void Actuator_assignment(const u8 *rx_buff, ACTUATOR_STATUS *Actuator_status);
void Motion_assignment(const u8 *rx_buff, MOTION_STATUS *Motion_status);
void PID_assignment(const u8 *rx_buff, PID_PARAS *PID_paras);

	

#endif
