//#include "write_protocol_variable.h"
#include <stdint.h>
#include "IMU.h"
#include "nrf_protocol.h"

#define HANDSHAKE_MARK 0xff

//#include "motor.h"

/*u8 Mode_init(SYS_STATUS *SYS_status)
{
	SYS_status->DTU_NRF_Status = 0x00;
	SYS_status->DTU_NRF_Status |= FAULT_MODE;										//初始化模式就行故障模式
	return 0x00;
}*/

void Manual_assignment(const u8 *rx_buff, MANUAL_STATUS *Manual_status)
{

	Manual_status->Yaw_Pulse = *(rx_buff + 1);
	Manual_status->Pitch_Pulse = *(rx_buff + 2);
	Manual_status->Roll_Pulse = *(rx_buff + 3);
	Manual_status->Fly_Pulse = *(rx_buff + 4);
	Manual_status->Climb_Pulse = *(rx_buff + 5);

}

void Flight_assignment(const u8 *rx_buff, FLIGHT_STATUS *Flight_status)
{

	Flight_status->x = *(rx_buff + 1);
	Flight_status->y = *(rx_buff + 2);
	Flight_status->z = *(rx_buff + 3);
	Flight_status->orientation = *(rx_buff + 4);
}

void Tuning_assignment(const u8 *rx_buff, PID_PARAS *PID_paras)
{
	PID_paras->PID_id = (uint8_t)*(rx_buff + 25 );//更新PID_id的值
/*	switch (PID_paras->PID_id)
	{
		case YAW:
		{
			PID_paras->PID_YPR_para[YAW].Kp_Int = *(rx_buff + 1);
			PID_paras->PID_YPR_para[YAW].Ki_Int = *(rx_buff + 2);
			PID_paras->PID_YPR_para[YAW].Kd_Int = *(rx_buff + 3);
			PID_paras->PID_YPR_para[YAW].Kp_Ext = *(rx_buff + 4);
			PID_paras->PID_YPR_para[YAW].Ki_Ext = *(rx_buff + 5);
			PID_paras->PID_YPR_para[YAW].Kd_Ext = *(rx_buff + 6);
			PID_paras->PID_YPR_para[YAW].SetPoint_Ext = *(int16_t *)&rx_buff[7];
		};
		break;
		case PITCH:
		{
			PID_paras->PID_YPR_para[PITCH].Kp_Int = *(rx_buff + 1);
			PID_paras->PID_YPR_para[PITCH].Ki_Int = *(rx_buff + 2);
			PID_paras->PID_YPR_para[PITCH].Kd_Int = *(rx_buff + 3);
			PID_paras->PID_YPR_para[PITCH].Kp_Ext = *(rx_buff + 4);
			PID_paras->PID_YPR_para[PITCH].Ki_Ext = *(rx_buff + 5);
			PID_paras->PID_YPR_para[PITCH].Kd_Ext = *(rx_buff + 6);
			PID_paras->PID_YPR_para[PITCH].SetPoint_Ext = *(int16_t *)&rx_buff[7];
		};
		break;
		case ROLL:
		{
			PID_paras->PID_YPR_para[ROLL].Kp_Int = *(rx_buff + 1);
			PID_paras->PID_YPR_para[ROLL].Ki_Int = *(rx_buff + 2);
			PID_paras->PID_YPR_para[ROLL].Kd_Int = *(rx_buff + 3);
			PID_paras->PID_YPR_para[ROLL].Kp_Ext = *(rx_buff + 4);
			PID_paras->PID_YPR_para[ROLL].Ki_Ext = *(rx_buff + 5);
			PID_paras->PID_YPR_para[ROLL].Kd_Ext = *(rx_buff + 6);
			PID_paras->PID_YPR_para[ROLL].SetPoint_Ext = *(int16_t *)&rx_buff[7];
		};
		break;
	}*/
	PID_paras->PID_YPR_para[YAW][RATE].Kp = *(rx_buff + 1);
	PID_paras->PID_YPR_para[YAW][RATE].Ki = *(rx_buff + 2);
	PID_paras->PID_YPR_para[YAW][RATE].Kd = *(rx_buff + 3);
	PID_paras->PID_YPR_para[YAW][ANGLE].Kp = *(rx_buff + 4);
	PID_paras->PID_YPR_para[YAW][ANGLE].Ki= *(rx_buff + 5);
	PID_paras->PID_YPR_para[YAW][ANGLE].Kd = *(rx_buff + 6);
	PID_paras->PID_YPR_para[PITCH][RATE].Kp = *(rx_buff + 7);
	PID_paras->PID_YPR_para[PITCH][RATE].Ki = *(rx_buff + 8);
	PID_paras->PID_YPR_para[PITCH][RATE].Kd = *(rx_buff + 9);
	PID_paras->PID_YPR_para[PITCH][ANGLE].Kp = *(rx_buff + 10);
	PID_paras->PID_YPR_para[PITCH][ANGLE].Ki = *(rx_buff + 11);
	PID_paras->PID_YPR_para[PITCH][ANGLE].Kd = *(rx_buff + 12);
	PID_paras->PID_YPR_para[ROLL][RATE].Kp = *(rx_buff + 13);
	PID_paras->PID_YPR_para[ROLL][RATE].Ki = *(rx_buff + 14);
	PID_paras->PID_YPR_para[ROLL][RATE].Kd = *(rx_buff + 15);
	PID_paras->PID_YPR_para[ROLL][ANGLE].Kp = *(rx_buff + 16);
	PID_paras->PID_YPR_para[ROLL][ANGLE].Ki = *(rx_buff + 17);
	PID_paras->PID_YPR_para[ROLL][ANGLE].Kd = *(rx_buff + 18);
	
	PID_paras->PID_YPR_para[YAW][ANGLE].SetPoint = *(int16_t *)&rx_buff[19];
	PID_paras->PID_YPR_para[ROLL][ANGLE].SetPoint = *(int16_t *)&rx_buff[21];
	PID_paras->PID_YPR_para[PITCH][ANGLE].SetPoint = *(int16_t *)&rx_buff[23];

	
}

void Fault_assignment(const u8 *rx_buff, SYS_STATUS *SYS_status)
{
	
}
/*===========================================================================
* 函数 ：START_assignment(const u8 *rx_buff, SYS_STATUS *SYS_status)                                   
* 输入 ：const u8 *rx_buff, SYS_STATUS *SYS_status
* 返回 ：无；		  				
* 说明 ：此函数是在START MODE下的解包函数；主要是实现更新当前模式的功能；
============================================================================*/
void Start_assignment(const u8 *rx_buff, SYS_STATUS *SYS_status)
{
	SYS_status->Manual_Status.Yaw_Pulse = *(rx_buff + 1);
	SYS_status->Manual_Status.Pitch_Pulse = *(rx_buff + 2);
	SYS_status->Manual_Status.Roll_Pulse = *(rx_buff + 3);
}


u8 Command_dispatch(u8 *rx_buff, SYS_STATUS *SYS_status)
{
	u8 pre_mode_ID = SYS_status->DTU_NRF_Status & 0x38;
	u8 current_mode_ID = *rx_buff;
	if(pre_mode_ID != current_mode_ID)
	{
		switch(pre_mode_ID)
		{
			case START_MODE:
			{
				SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|current_mode_ID;		//更新模式
			}break;			
			case FAULT_MODE:
			{
				if(current_mode_ID == START_MODE)
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|current_mode_ID;	//更新模式
				}
				else
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|FAULT_MODE;		//故障模式只能重新从START_MODE开始，否则还是故障模式
				}
			}break;
			default:
			{
				if(current_mode_ID == START_MODE)
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|current_mode_ID;	//更新模式
				}
				else
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|FAULT_MODE;		//其他的运行模式只能重新从START_MODE开始，否则还是故障模式
				}
			}
		}
	}
	
	switch (current_mode_ID)
	{
		case MANUAL_MODE:
			Manual_assignment(rx_buff, &(SYS_status->Manual_Status));
			break;
		case FLIGHT_MODE:
			Flight_assignment(rx_buff, &(SYS_status->Flight_Status));
			break;
		case TUNING_MODE:
			Tuning_assignment(rx_buff, &(SYS_status->PID_Paras));
			break;
		case START_MODE:
			Start_assignment(rx_buff, SYS_status);
		case FAULT_MODE:
			Fault_assignment(rx_buff, SYS_status);
			break;
	}
	return current_mode_ID;
}

void Command_patch(u8 *tx_buff, SYS_STATUS *SYS_status, u8 mode_id)
{
	u8 pid_id = SYS_status->PID_Paras.PID_id;
	if (mode_id == START_MODE)
	{
		*tx_buff = START_MODE;
		*(u16 *)&tx_buff[1] = (u16)((SYS_status->imu_fusion_module.ypr[YAW]) * 100);					//将角度值*100传到上位机
		*(u16 *)&tx_buff[3] = (u16)((SYS_status->imu_fusion_module.ypr[PITCH]) * 100);
		*(u16 *)&tx_buff[5] = (u16)((SYS_status->imu_fusion_module.ypr[ROLL]) * 100);
		*(u16 *)&tx_buff[7] = (u16)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(u16 *)&tx_buff[9] = (u16)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(u16 *)&tx_buff[11] = (u16)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = SYS_status->Manual_Status.Yaw_Pulse;
		*(tx_buff + 19) = SYS_status->Manual_Status.Pitch_Pulse;
		*(tx_buff + 20) = SYS_status->Manual_Status.Roll_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
	}
	if(mode_id == MANUAL_MODE)
	{
		*tx_buff = MANUAL_MODE;
		*(u16 *)&tx_buff[1] = (u16)((SYS_status->imu_fusion_module.ypr[YAW]) * 100);					//将角度值*100传到上位机
		*(u16 *)&tx_buff[3] = (u16)((SYS_status->imu_fusion_module.ypr[PITCH]) * 100);
		*(u16 *)&tx_buff[5] = (u16)((SYS_status->imu_fusion_module.ypr[ROLL]) * 100);
		*(u16 *)&tx_buff[7] = (u16)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(u16 *)&tx_buff[9] = (u16)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(u16 *)&tx_buff[11] = (u16)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = SYS_status->Manual_Status.Yaw_Pulse;
		*(tx_buff + 19) = SYS_status->Manual_Status.Pitch_Pulse;
		*(tx_buff + 20) = SYS_status->Manual_Status.Roll_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
	}
	if(mode_id == FLIGHT_MODE)
	{
		*tx_buff = FLIGHT_MODE;
		*(u16 *)&tx_buff[1] = (u16)((SYS_status->imu_fusion_module.ypr[YAW]) * 100);					//将角度值*100传到上位机
		*(u16 *)&tx_buff[3] = (u16)((SYS_status->imu_fusion_module.ypr[PITCH]) * 100);
		*(u16 *)&tx_buff[5] = (u16)((SYS_status->imu_fusion_module.ypr[ROLL]) * 100);
		*(u16 *)&tx_buff[7] = (u16)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(u16 *)&tx_buff[9] = (u16)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(u16 *)&tx_buff[11] = (u16)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = SYS_status->Manual_Status.Yaw_Pulse;
		*(tx_buff + 19) = SYS_status->Manual_Status.Pitch_Pulse;
		*(tx_buff + 20) = SYS_status->Manual_Status.Roll_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
	}
	if(mode_id == TUNING_MODE)
	{
		*tx_buff = TUNING_MODE;
		*(u16 *)&tx_buff[1] = (u16)((SYS_status->imu_fusion_module.ypr[YAW]) * 100);					//将角度值*100传到上位机
		*(u16 *)&tx_buff[3] = (u16)((SYS_status->imu_fusion_module.ypr[PITCH]) * 100);
		*(u16 *)&tx_buff[5] = (u16)((SYS_status->imu_fusion_module.ypr[ROLL]) * 100);
		*(u16 *)&tx_buff[7] = (u16)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(u16 *)&tx_buff[9] = (u16)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(u16 *)&tx_buff[11] = (u16)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = SYS_status->Manual_Status.Yaw_Pulse;
		*(tx_buff + 19) = SYS_status->Manual_Status.Pitch_Pulse;
		*(tx_buff + 20) = SYS_status->Manual_Status.Roll_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
		*(u16 *)&tx_buff[23] = (u16)(SYS_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Error);
		*(u16 *)&tx_buff[25] = (u16)(SYS_status->PID_Paras.PID_YPR_para[pid_id][RATE].Error);
		*(tx_buff + 27) = SYS_status->PID_Paras.PID_id;
		
	}
	if(mode_id == FAULT_MODE)
	{
		*tx_buff = FAULT_MODE;
		*(tx_buff + 1) = SYS_status->DTU_NRF_Status;
	}
}
