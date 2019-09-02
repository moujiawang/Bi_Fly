//#include "write_protocol_variable.h"
#include <stdint.h>
#include "IMU.h"
#include "nrf_protocol.h"
#include <stdint.h>


//#define HANDSHAKE_MARK 0xff

//#include "motor.h"

/*u8 Mode_init(SYS_STATUS *SYS_status)
{
	SYS_status->DTU_NRF_Status = 0x00;
	SYS_status->DTU_NRF_Status |= FAULT_MODE;										//初始化模式就行故障模式
	return 0x00;
}*/

void Manual_assignment(const u8 *rx_buff, MANUAL_STATUS *Manual_status)
{
	Manual_status->MidServo_Pulse = (float)*(rx_buff + 1);
	Manual_status->LeftServo_Pulse = (float)*(rx_buff + 2);
	Manual_status->RightServo_Pulse =(float) *(rx_buff + 3);
	Manual_status->Climb_Pulse = *(rx_buff + 4);
	Manual_status->Fly_Pulse = *(rx_buff + 5);
}

void Flight_assignment(const u8 *rx_buff, FLIGHT_STATUS *Flight_status)
{
	Flight_status->x = *(rx_buff + 1);
	Flight_status->y = *(rx_buff + 2);
	Flight_status->z = *(rx_buff + 3);
	Flight_status->orientation = *(rx_buff + 4);
}

void Tuning_assignment(const u8 *rx_buff, SYS_STATUS *sys_status)
{
	u8 pid_id = 0;
	 pid_id = *(rx_buff + 14 );//更新PID_id的值
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
	sys_status->PID_Paras.PID_YPR_para[pid_id][RATE].Kp = (float)(*(uint16_t *)&rx_buff[1])/100;
	sys_status->PID_Paras.PID_YPR_para[pid_id][RATE].Ki = (float)(*(uint16_t *)&rx_buff[3])/100;
	sys_status->PID_Paras.PID_YPR_para[pid_id][RATE].Kd = (float)(*(uint16_t *)&rx_buff[5])/100;
	sys_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Kp = (float)(*(uint16_t *)&rx_buff[7])/100;
	sys_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Ki = (float)(*(uint16_t *)&rx_buff[9])/100;
	sys_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Kd = (float)(*(uint16_t *)&rx_buff[11])/100;
	sys_status->PID_Paras.refresh_Hz = (uint16_t)(1000 / (*(rx_buff + 13)));

	sys_status->PID_Paras.PID_id = pid_id;
	sys_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].LimitLow = (float)(*(int16_t *)&rx_buff[15]) / 10;
	sys_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Limit = (float)(*(uint16_t *)&rx_buff[17]) / 10;	


	sys_status->Manual_Status.RightServo_LimitLow = *(uint8_t *)&rx_buff[19] ;
	sys_status->Manual_Status.LeftServo_LimitLow = *(uint8_t *)&rx_buff[19] ;
	sys_status->Manual_Status.MidServo_LimitLow = *(uint8_t *)&rx_buff[19];
	sys_status->Manual_Status.RightServo_Limit = *(uint8_t *)&rx_buff[20] ;
	sys_status->Manual_Status.LeftServo_Limit = *(uint8_t *)&rx_buff[20] ;
	sys_status->Manual_Status.MidServo_Limit = *(uint8_t *)&rx_buff[20] ;

	sys_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].SetPoint = (float)(*(int16_t *)&rx_buff[21]) / 100;
	sys_status->Manual_Status.Fly_Pulse = *(rx_buff + 23);
	

	
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
	SYS_status->Manual_Status.RightServo_Pulse = (float)*(rx_buff + 1);
	SYS_status->Manual_Status.LeftServo_Pulse = (float)*(rx_buff + 2);
	SYS_status->Manual_Status.MidServo_Pulse = (float)*(rx_buff + 3);
	SYS_status->Manual_Status.Fly_Pulse = *(rx_buff + 4);
}

/*===========================================================================
* 函数 ：Command_dispatch(u8 *rx_buff, SYS_STATUS *SYS_status, u8 rx_len)                                  
* 输入 ：u8 *rx_buff, SYS_STATUS *SYS_status, u8 rx_len
* 返回 ：(1)0xff		  				
* 说明 ：此函数是在START MODE下的解包函数；主要是实现更新当前模式的功能；
============================================================================*/

u8 Command_dispatch(u8 *rx_buff, SYS_STATUS *SYS_status, u8 rx_len)
{
	u8 pre_mode_ID = SYS_status->DTU_NRF_Status & 0x38;
	u8 current_mode_ID = *rx_buff & 0x38;
	switch(current_mode_ID)
	{
		case MANUAL_MODE:
			if(rx_len != MANUAL_MODE_DOWNDATA_LENGTH)
				return 0xff;
			break;
		case FLIGHT_MODE:
			if(rx_len != FLIGHT_MODE_DOWNDATA_LENGTH)
				return 0xff;
			break;
		case TUNING_MODE:
			if(rx_len != TUNING_MODE_DOWNDATA_LENGTH)
				return 0xff;
			break;
		case START_MODE:
			if(rx_len != START_MODE_DOWNDATA_LENGTH)
				return 0xff;
			break;
		case FAULT_MODE:
		
			if(rx_len != FAULT_MODE_DOWNDATA_LENGTH)
				return 0xff;
			break;
		default: 
			return 0xff;			
	}
	if(*rx_buff & 0x01)
	{
		SYS_status->DTU_NRF_Status |= WRITE_FLASH_FLAG;		
	}
	//模式判断：如果当前模式是fault mode，那可以调整到start mode,不能直接从fault mode 转换到其他的模式，如果当前是start mode，那可以转换到其他任何模式
	//			如果是除了start mode fault mode以外的其他模式，那只能回到fault mode或者start mode
	if(pre_mode_ID != current_mode_ID)
	{
		switch(pre_mode_ID)
		{
			case START_MODE:
			{
				SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|current_mode_ID;		//更新模式
				if(current_mode_ID == TUNING_MODE)
				{
					SYS_status->Manual_Status.LeftServo_Out_Sum = SYS_status->Manual_Status.LeftServo_Pulse;
					SYS_status->Manual_Status.RightServo_Out_Sum = SYS_status->Manual_Status.RightServo_Pulse;
					SYS_status->Manual_Status.MidServo_Out_Sum = SYS_status->Manual_Status.MidServo_Pulse;
				}
			}break;			
			case FAULT_MODE:
			{
				if(current_mode_ID == START_MODE)
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|START_MODE;	//更新模式
				}
				else
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|FAULT_MODE;	//故障模式只能重新从START_MODE开始，否则还是故障模式
				}
			}break;
			default:
			{
				if(current_mode_ID == START_MODE)
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|START_MODE;	//更新模式
					IMU_status_init(SYS_status);		//初始化IMU所有的状态量													
				}
				else
				{
					SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|FAULT_MODE;		//其他的运行模式只能重新从START_MODE开始，否则还是故障模式
				}
			}
		}
	}
	
	switch (SYS_status->DTU_NRF_Status & 0x38)
	{
		case MANUAL_MODE:
			Manual_assignment(rx_buff, &(SYS_status->Manual_Status));
			break;
		case FLIGHT_MODE:
			Flight_assignment(rx_buff, &(SYS_status->Flight_Status));
			break;
		case TUNING_MODE:
			Tuning_assignment(rx_buff, SYS_status);
			break;
		case START_MODE:
			Start_assignment(rx_buff, SYS_status);
			break;
		case FAULT_MODE:
			Fault_assignment(rx_buff, SYS_status);
			break;
	}
	return current_mode_ID;
}

u8 Command_patch(u8 *tx_buff, SYS_STATUS *SYS_status, u8 mode_id)
{
	u8 pid_id = SYS_status->PID_Paras.PID_id;
	u8 length = 0;
	if (mode_id == START_MODE)
	{
		*tx_buff = START_MODE;
		*(int16_t *)&tx_buff[1] = (int16_t)(SYS_status->imu_fusion_module.ypr[YAW]*100);					//将角度值*100传到上位机
		*(int16_t *)&tx_buff[3] = (int16_t)(SYS_status->imu_fusion_module.ypr[PITCH]*100);
		*(int16_t *)&tx_buff[5] = (int16_t)(SYS_status->imu_fusion_module.ypr[ROLL]*100);
		*(int16_t *)&tx_buff[7] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(int16_t *)&tx_buff[9] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(int16_t *)&tx_buff[11] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = (u8)SYS_status->Manual_Status.MidServo_Pulse;
		*(tx_buff + 19) = (u8)SYS_status->Manual_Status.LeftServo_Pulse;
		*(tx_buff + 20) = (u8)SYS_status->Manual_Status.RightServo_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
		length = START_MODE_UPDATA_LENGTH;
	}
	if(mode_id == MANUAL_MODE)
	{
		*tx_buff = MANUAL_MODE;
		*(int16_t *)&tx_buff[1] = (int16_t)(SYS_status->imu_fusion_module.ypr[YAW]*100);					//将角度值*100传到上位机
		*(int16_t *)&tx_buff[3] = (int16_t)(SYS_status->imu_fusion_module.ypr[PITCH]*100);
		*(int16_t *)&tx_buff[5] = (int16_t)(SYS_status->imu_fusion_module.ypr[ROLL]*100);
		*(int16_t *)&tx_buff[7] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(int16_t *)&tx_buff[9] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(int16_t *)&tx_buff[11] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = (u8)SYS_status->Manual_Status.MidServo_Pulse;
		*(tx_buff + 19) = (u8)SYS_status->Manual_Status.LeftServo_Pulse;
		*(tx_buff + 20) = (u8)SYS_status->Manual_Status.RightServo_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
		length = MANUAL_MODE_UPDATA_LENGTH;
	}
	if(mode_id == FLIGHT_MODE)
	{
		*tx_buff = FLIGHT_MODE;
		*(int16_t *)&tx_buff[1] = (int16_t)((SYS_status->imu_fusion_module.ypr[YAW]) * 100);					//将角度值*100传到上位机
		*(int16_t *)&tx_buff[3] = (int16_t)((SYS_status->imu_fusion_module.ypr[PITCH]) * 100);
		*(int16_t *)&tx_buff[5] = (int16_t)((SYS_status->imu_fusion_module.ypr[ROLL]) * 100);
		*(int16_t *)&tx_buff[7] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(int16_t *)&tx_buff[9] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(int16_t *)&tx_buff[11] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = (u8)SYS_status->Manual_Status.MidServo_Pulse;
		*(tx_buff + 19) = (u8)SYS_status->Manual_Status.LeftServo_Pulse;
		*(tx_buff + 20) = (u8)SYS_status->Manual_Status.RightServo_Pulse;
		*(tx_buff + 21) = SYS_status->Manual_Status.Fly_Pulse;
		*(tx_buff + 22) = SYS_status->Manual_Status.Climb_Pulse;
		length = FLIGHT_MODE_UPDATA_LENGTH;
	}
	if(mode_id == TUNING_MODE)
	{
		*tx_buff = TUNING_MODE;
		*(int16_t *)&tx_buff[1] = (int16_t)(SYS_status->imu_fusion_module.ypr[pid_id]*100);					//将角度值*100传到上位机
		*(int16_t *)&tx_buff[3] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[pid_id]);
		*(uint32_t *)&tx_buff[5] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 9) = SYS_status->DTU_NRF_Status;
		*(int16_t *)&tx_buff[10] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Error * 100);			//将角度误差乘以100上发	
		*(int16_t *)&tx_buff[12] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][RATE].SetPoint * 10);				//将角速度误差乘以10上发
		*(int16_t *)&tx_buff[14] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Kp_OUT_SUM * 100);	
		*(int16_t *)&tx_buff[16] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Ki_OUT_SUM * 100);	
		*(int16_t *)&tx_buff[18]= (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][ANGLE].Kd_OUT_SUM * 100);	
		*(int16_t *)&tx_buff[20] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][RATE].Kp_OUT_SUM * 100);	
		*(int16_t *)&tx_buff[22] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][RATE].Ki_OUT_SUM * 100);	
		*(int16_t *)&tx_buff[24] = (int16_t)(SYS_status->PID_Paras.PID_YPR_para[pid_id][RATE].Kd_OUT_SUM * 100);
		*(tx_buff + 26) = SYS_status->PID_Paras.PID_id;
		*(tx_buff + 27) = (u8)SYS_status->Manual_Status.RightServo_Pulse;
		*(tx_buff + 28) = (u8)SYS_status->Manual_Status.LeftServo_Pulse;
		*(tx_buff + 29) = (u8)SYS_status->Manual_Status.MidServo_Pulse;		
		length = TUNING_MODE_UPDATA_LENGTH;
		
	}
	if(mode_id == FAULT_MODE)
	{
		*tx_buff = FAULT_MODE;
		*(int16_t *)&tx_buff[1] = (int16_t)(SYS_status->imu_fusion_module.ypr[YAW]*100);					//将角度值*100传到上位机
		*(int16_t *)&tx_buff[3] = (int16_t)(SYS_status->imu_fusion_module.ypr[PITCH]*100);
		*(int16_t *)&tx_buff[5] = (int16_t)(SYS_status->imu_fusion_module.ypr[ROLL]*100);
		*(int16_t *)&tx_buff[7] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[YAW]);
		*(int16_t *)&tx_buff[9] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[PITCH]);
		*(int16_t *)&tx_buff[11] = (int16_t)(SYS_status->imu_fusion_module.ypr_rate[ROLL]);
		*(uint32_t *)&tx_buff[13] = SYS_status->imu_fusion_module.system_micrsecond;
		*(tx_buff + 17) = SYS_status->DTU_NRF_Status;
		*(tx_buff + 18) = (u8)SYS_status->Manual_Status.MidServo_Pulse;
		*(tx_buff + 19) = (u8)SYS_status->Manual_Status.LeftServo_Pulse;
		*(tx_buff + 20) = (u8)SYS_status->Manual_Status.RightServo_Pulse;
		length = FAULT_MODE_UPDATA_LENGTH;
	}
	return length;
	
}
