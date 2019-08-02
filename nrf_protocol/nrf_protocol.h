#ifndef NRF_PROTOCOL_H
#define NRF_PROTOCOL_H

#include <stdint.h>
#include "stm32f10x.h"



typedef enum{ TX_MODE, RX_MODE } NRF24L01_MODE; 		//TX_MODE->0,RX_MODE->1
typedef enum{ PID_YAW, PID_PITCH,PID_ROLL } PID_ID;     //PID_YAW->0, PID_PITCH->1,PID_ROLL->2
typedef enum{ TX_WAIT, TX_GO } TX_FLAG;					//Tx_Flag
typedef enum{ YAW, PITCH, ROLL, ALL} YPR_ID;			//YAW->0, PITCH->1,ROLL->2, ALL->3
typedef enum{ RATE, ANGLE } R_A_ID;						//RATE->0, ANGLE->1
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
	uint8_t Roll_Pulse;
	uint8_t Pitch_Pulse;
	uint8_t Yaw_Pulse;
	uint8_t Fly_Pulse;
	uint8_t Climb_Pulse;
	uint8_t Control_Status;
	uint8_t Fly_or_Climb_Status;
	u8 test;
}MANUAL_STATUS;

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
}FLIGHT_STATUS;

typedef struct
{
	float Error;			//e[k]
	float PreError;			//e[k-1]
	float LastError;		//e[k-2]
	float Kp;					
	float Ki;
	float Kd;
	float Kp_OUT;			//Kp_OUT(debugging)			
	float Ki_OUT;			//Ki_OUT(debugging)	
	float Kd_OUT;			//Kd_OUT(debugging)	
	float PIDcal_Out;
	float SetPoint;
	float Limit;       		//output limit MAX
	float LimitLow;    		//output limit MIN
}PID_PARA;

typedef struct
{
	YPR_ID PID_id;
	PID_PARA PID_YPR_para[3][2];		
}PID_PARAS;

typedef struct
{
	u8 DTU_NRF_Status;
	IMUFusion imu_fusion_module;
	MANUAL_STATUS Manual_Status;
	FLIGHT_STATUS Flight_Status;
	PID_PARAS PID_Paras;
}SYS_STATUS;






//NRF24L01 和 接收机是否在线标志号

#define NRF_ON            0x01					//NRF在线，表明飞行器端有可以与主控芯进行通讯的NRF模块
#define NRF_OFF           0xfe					//NRF不在线

//NRF24L01和接收机通讯标志号
#define NRF_CONNECTED     0x02					//NRF通讯成功
#define NRF_DISCONNECTED  0xFD					//NRF通讯失败
#define DTU_OK            0x03					//DTU接收机通讯正常就表明在线，DTU无是否在线的检测标志位



#define MODE_STATUS (SYS_Status.DTU_NRF_Status & 0x38) 

//u8 Mode_init(SYS_STATUS *SYS_status);
u8 Command_dispatch(u8 *rx_buff, SYS_STATUS *SYS_status);
void Command_patch(u8 *tx_buff, SYS_STATUS *SYS_status, u8 mode_id);
void Manual_assignment(const u8 *rx_buff, MANUAL_STATUS *Actuator_status);
void Flight_assignment(const u8 *rx_buff, FLIGHT_STATUS *Motion_status);
void Tuning_assignment(const u8 *rx_buff, PID_PARAS *PID_paras);
void Start_assignment(const u8 *rx_buff, SYS_STATUS *SYS_status);
	

#endif
