#include "write_protocol_variable.h"
#include <stdint.h>
#include "IMU.h"
#include "nrf_protocol.h"
//#include "motor.h"

void Actuator_assignment(const u8 *rx_buff, ACTUATOR_STATUS *Actuator_status)
{
	//    ACTUATOR_STATUS temp_basic_control;
	//    temp_basic_control.lift  = *(uint16_t*)&data_address[0];
	//    temp_basic_control.yaw   = *( int16_t*)&data_address[2];
	//    temp_basic_control.pitch = *( int16_t*)&data_address[4];
	//    temp_basic_control.roll  = *( int16_t*)&data_address[6];
	//    SetActuatorControl(&temp_basic_control);

	Actuator_status->Fly_Pulse = *rx_buff;
	Actuator_status->Climb_Pulse = *(rx_buff + 1);
	Actuator_status->Pitch_Pulse = *(rx_buff + 2);
	Actuator_status->Roll_Pulse = *(rx_buff + 3);
	Actuator_status->Yaw_Pulse = *(rx_buff + 4);
}

void Motion_assignment(const u8 *rx_buff, MOTION_STATUS *Motion_status)
{
	//    MOTION_STATUS temp_motion_control;
	//    temp_motion_control.x   = *(int16_t*)&data_address[0];
	//    temp_motion_control.y   = *(int16_t*)&data_address[2];
	//    temp_motion_control.z   = *(int16_t*)&data_address[4];
	//    temp_motion_control.orientation = *(int16_t*)&data_address[6];
	//    SetMotionControl(&temp_motion_control);

	Motion_status->x = *rx_buff;
	Motion_status->y = *(rx_buff + 1);
	Motion_status->z = *(rx_buff + 2);
	Motion_status->orientation = *(rx_buff + 3);
}

void PID_assignment(const u8 *rx_buff, PID_PARAS *PID_paras)
{
	PID_paras->PID_id = *rx_buff;//更新PID_id的值
	switch (PID_paras->PID_id)
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
	}
}
/*===========================================================================
* 函数 ：START_assignment(const u8 *rx_buff, SYS_STATUS *SYS_status)                                    * 
* 输入 ：const u8 *rx_buff, SYS_STATUS *SYS_status
* 返回 ：无；		  				
* 说明 ：此函数是在START MODE下的解包函数；主要是实现更新当前模式的功能；
============================================================================*/
void START_assignment(const u8 *rx_buff, SYS_STATUS *SYS_status)
{
	SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7)|rx_buff[1];		//更新模式信息
}


u8 Command_dispatch(u8 *rx_buff, SYS_STATUS *SYS_status)
{
	uint16_t Mode_ID = *rx_buff;
	switch (Mode_ID)
	{
		case ACTUATOR_MODE:
			Actuator_assignment(rx_buff + 1, &(SYS_status->Actuator_Status));
			break;
		case MOTION_MODE:
			Motion_assignment(rx_buff + 1,  &(SYS_status->Motion_Status));
			break;
		case PID_MODE:
			PID_assignment(rx_buff + 1, &(SYS_status->PID_Paras));
			break;
		case START_MODE:
//			START_assignment(rx_buff + 1, SYS_status);
			break;
	}
	return Mode_ID;
}

void Command_patch(u8 *tx_buff, SYS_STATUS *SYS_status, u8 mode_id)
{

	if (mode_id == START_MODE)
	{
		*tx_buff = 0xAA;
<<<<<<< HEAD
		*(tx_buff + 1) = PID_paras->PID_Yaw_para.Kp_Int;
		*(tx_buff + 2) = PID_paras->PID_Yaw_para.Ki_Int;
		*(tx_buff + 3) = PID_paras->PID_Yaw_para.Kd_Int;
		*(tx_buff + 4) = PID_paras->PID_Yaw_para.Kp_Ext;
		*(tx_buff + 5) = PID_paras->PID_Yaw_para.Ki_Ext;
		*(tx_buff + 6) = PID_paras->PID_Yaw_para.Kd_Ext;
		*(tx_buff + 7) = PID_paras->PID_Pitch_para.Kp_Int;
		*(tx_buff + 8) = PID_paras->PID_Pitch_para.Ki_Int;
		*(tx_buff + 9) = PID_paras->PID_Pitch_para.Kd_Int;
		*(tx_buff + 10) = PID_paras->PID_Pitch_para.Kp_Ext;
		*(tx_buff + 11) = PID_paras->PID_Pitch_para.Ki_Ext;
		*(tx_buff + 12) = PID_paras->PID_Pitch_para.Kd_Ext;
		*(tx_buff + 13) = PID_paras->PID_Roll_para.Kp_Int;
		*(tx_buff + 14) = PID_paras->PID_Roll_para.Ki_Int;
		*(tx_buff + 15) = PID_paras->PID_Roll_para.Kd_Int;
		*(tx_buff + 16) = PID_paras->PID_Roll_para.Kp_Ext;
		*(tx_buff + 17) = PID_paras->PID_Roll_para.Ki_Ext;
		*(tx_buff + 18) = PID_paras->PID_Roll_para.Kd_Ext;
		*(u16 *)&tx_buff[19] = (u16)((Attitude->ypr[0]) * 100);
		*(u16 *)&tx_buff[21] = (u16)((Attitude->ypr[1]) * 100);
		*(u16 *)&tx_buff[23] = (u16)((Attitude->ypr[2]) * 100);
		*(uint32_t *)&tx_buff[25] = Attitude->system_micrsecond;
		*(tx_buff + 29) = 
=======
		*(tx_buff + 1) = SYS_status->PID_Paras.PID_YPR_para[YAW].Kp_Int;
		*(tx_buff + 2) = SYS_status->PID_Paras.PID_YPR_para[YAW].Ki_Int;
		*(tx_buff + 3) = SYS_status->PID_Paras.PID_YPR_para[YAW].Kd_Int;
		*(tx_buff + 4) = SYS_status->PID_Paras.PID_YPR_para[YAW].Kp_Ext;
		*(tx_buff + 5) = SYS_status->PID_Paras.PID_YPR_para[YAW].Ki_Ext;
		*(tx_buff + 6) = SYS_status->PID_Paras.PID_YPR_para[YAW].Kd_Ext;
		*(tx_buff + 7) = SYS_status->PID_Paras.PID_YPR_para[PITCH].Kp_Int;
		*(tx_buff + 8) = SYS_status->PID_Paras.PID_YPR_para[PITCH].Ki_Int;
		*(tx_buff + 9) = SYS_status->PID_Paras.PID_YPR_para[PITCH].Kd_Int;
		*(tx_buff + 10) = SYS_status->PID_Paras.PID_YPR_para[PITCH].Kp_Ext;
		*(tx_buff + 11) = SYS_status->PID_Paras.PID_YPR_para[PITCH].Ki_Ext;
		*(tx_buff + 12) = SYS_status->PID_Paras.PID_YPR_para[PITCH].Kd_Ext;
		*(tx_buff + 13) = SYS_status->PID_Paras.PID_YPR_para[ROLL].Kp_Int;
		*(tx_buff + 14) = SYS_status->PID_Paras.PID_YPR_para[ROLL].Ki_Int;
		*(tx_buff + 15) = SYS_status->PID_Paras.PID_YPR_para[ROLL].Kd_Int;
		*(tx_buff + 16) = SYS_status->PID_Paras.PID_YPR_para[ROLL].Kp_Ext;
		*(tx_buff + 17) = SYS_status->PID_Paras.PID_YPR_para[ROLL].Ki_Ext;
		*(tx_buff + 18) = SYS_status->PID_Paras.PID_YPR_para[ROLL].Kd_Ext;
		*(u16 *)&tx_buff[19] = (u16)((SYS_status->imu_fusion_module.ypr[0]) * 100);					//将角度值*100传到上位机
		*(u16 *)&tx_buff[21] = (u16)((SYS_status->imu_fusion_module.ypr[1]) * 100);
		*(u16 *)&tx_buff[23] = (u16)((SYS_status->imu_fusion_module.ypr[2]) * 100);
		*(uint32_t *)&tx_buff[25] = SYS_status->imu_fusion_module.system_micrsecond;
>>>>>>> 67753ccbe8b87348627ef750ab8053a7a981ca88
	}
	else
	{
		*tx_buff = 0xB0;											//上行报文ID
		*(tx_buff + 1) = SYS_status->Actuator_Status.Fly_Pulse;
		*(tx_buff + 2) = SYS_status->Actuator_Status.Climb_Pulse;
		*(tx_buff + 3) = SYS_status->Actuator_Status.Pitch_Pulse;
		*(tx_buff + 4) = SYS_status->Actuator_Status.Roll_Pulse;
		*(tx_buff + 5) = SYS_status->Actuator_Status.Yaw_Pulse;

		*(u16 *)&tx_buff[6]  = 0;
		*(u16 *)&tx_buff[8]  = 0;
		*(u16 *)&tx_buff[10] = 0;
		*(u16 *)&tx_buff[12] = (u16)((SYS_status->imu_fusion_module.ypr[0]) * 100);
		*(u16 *)&tx_buff[14] = (u16)((SYS_status->imu_fusion_module.ypr[1]) * 100);
		*(u16 *)&tx_buff[16] = (u16)((SYS_status->imu_fusion_module.ypr[2]) * 100);
		*(uint32_t *)&tx_buff[18]  = SYS_status->imu_fusion_module.system_micrsecond;
	}
}
