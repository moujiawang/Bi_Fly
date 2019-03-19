#ifndef NRF_PROTOCOL_H
#define NRF_PROTOCOL_H

#include <stdint.h>
#include "stm32f10x.h"

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
	uint8_t Kp_Int;					
	uint8_t Ki_Int;
	uint8_t Kd_Int;
	uint8_t Kp_Ext;
	uint8_t Ki_Ext;
	uint8_t Kd_Ext;
	int8_t PIDcal_ExtOut;
	int16_t PIDcal_IntOut;
	int16_t SetPoint_Int;
	int16_t SetPoint_Ext;
}PID_PARA;

typedef struct
{
	PID_ID PID_id;
	PID_PARA PID_Yaw_para;		
	PID_PARA PID_Pitch_para;
	PID_PARA PID_Roll_para;
}PID_PARAS;

typedef struct
{
	u8 DTU_NRF_Status;
	
}SYS_STATUS;


//NRF24L01 和 接收机是否在线标志号

#define NRF_ON            0x01					//NRF在线，表明飞行器端有可以与主控芯进行通讯的NRF模块
#define NRF_OFF           0xfe					//NRF不在线

//NRF24L01和接收机通讯标志号
#define NRF_CONNECTED     0x02	
#define NRF_DISCONNECTED  0xFD					//NRF通讯失败
#define DTU_OK            0x03					//DTU接收机通讯正常就表明在线，DTU无是否在线的检测标志位


//模式标志号
#define START_MODE    0x00						//0000 0000
#define MOTION_MODE   0x08						//0000 1000
#define PID_MODE      0x10						//0001 0000
#define ACTUATOR_MODE 0x18						//0001 1000
#define FAULT_MODE    0x38						//0011 1000
 

#define MODE_STATUS (SYS_status.DTU_NRF_Status|0x38)

typedef enum{ TX_MODE, RX_MODE } NRF24L01_MODE; 		//TX_MODE->0,RX_MODE->1
typedef enum{ PID_YAW, PID_PITCH,PID_ROLL } PID_ID;     //PID_YAW->0, PID_PITCH->1,PID_ROLL->2
typedef enum{ TX_WAIT, TX_GO } TX_FLAG;					//Tx_Flag

u8 Command_dispatch(u8 *rx_buff, ACTUATOR_STATUS *Actuator_status, MOTION_STATUS *Motion_status, PID_PARAS *PID_paras，SYS_STATUS *SYS_status);
void Command_patch(u8 *tx_buff, PID_PARAS *PID_paras, ACTUATOR_STATUS* Actuator_Status, IMUFusion* Attitude, u8 mode_id);
void Actuator_assignment(const u8 *rx_buff, ACTUATOR_STATUS *Actuator_status);
void Motion_assignment(const u8 *rx_buff, MOTION_STATUS *Motion_status);
void PID_assignment(const u8 *rx_buff, PID_PARAS *PID_paras);

	

#endif
