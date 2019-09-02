#include "attitude_pid.h"
#include "IncPID.h"



/**************************************************************/
//函数名：attitudeRatePID 角速度PID环
//输  入：PID_PARAS（PID的参数） , error
//输  出：角度环PID计算输出，可以直接得到控制舵机电机占空比的值
//说  明：此程序是完成角速度环PID计算值
/**************************************************************/
void attitudeRatePID(PID_PARAS * pid_paras, IMUFusion * imu_data)	/* 角速度环PID */
{
	PID_Update(&pid_paras->PID_YPR_para[YAW][RATE], pid_paras->PID_YPR_para[YAW][RATE].SetPoint - imu_data->ypr_rate[YAW]);
	PID_Update(&pid_paras->PID_YPR_para[PITCH][RATE], pid_paras->PID_YPR_para[PITCH][RATE].SetPoint - imu_data->ypr_rate[PITCH]);
	PID_Update(&pid_paras->PID_YPR_para[ROLL][RATE], pid_paras->PID_YPR_para[ROLL][RATE].SetPoint - imu_data->ypr_rate[ROLL]);
}

/**************************************************************/
//函数名：attitudeRatePID 角度PID环
//输  入：PID_PARAS（PID的参数） , error
//输  出：角环PID计算输出，得到的输出为理想的角速度值，为后面的角速度环提供输入
//说  明：此程序是完成角度环PID计算值
/**************************************************************/
void attitudeAnglePID(PID_PARAS * pid_paras, IMUFusion * imu_data)	/* 角度环PID */
{
	float yawError =  0;
	pid_paras->PID_YPR_para[PITCH][RATE].SetPoint = PID_Update(&pid_paras->PID_YPR_para[PITCH][ANGLE], pid_paras->PID_YPR_para[PITCH][ANGLE].SetPoint - imu_data->ypr[PITCH]);
	pid_paras->PID_YPR_para[ROLL][RATE].SetPoint = PID_Update(&pid_paras->PID_YPR_para[ROLL][ANGLE], pid_paras->PID_YPR_para[ROLL][ANGLE].SetPoint - imu_data->ypr[ROLL]);

	yawError = pid_paras->PID_YPR_para[YAW][ANGLE].SetPoint -  imu_data->ypr[YAW];
	if(yawError > 180.0f) 
		yawError -= 360.0f;
	else if( yawError < -180.0 ) 
		yawError += 360.0f;
	pid_paras->PID_YPR_para[YAW][RATE].SetPoint = PID_Update(&pid_paras->PID_YPR_para[YAW][ANGLE], yawError);
}


void PID_command(SYS_STATUS *SYS_status)
{
	u8 pid_id;
	pid_id = SYS_status->PID_Paras.PID_id;
	
	attitudeAnglePID(&SYS_status->PID_Paras,&SYS_status->imu_fusion_module);// 角度环PID
	
	if( SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PIDcal_Out_SUM >= SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Limit )
		SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].SetPoint = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Limit;
	if( SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PIDcal_Out_SUM <= SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].LimitLow )
		SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].SetPoint = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].LimitLow;
	
	if( SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out_SUM >= SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Limit )
		SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].SetPoint = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Limit;
	if( SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out_SUM <= SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].LimitLow )
		SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].SetPoint = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].LimitLow;
	
	if( SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out_SUM >= SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Limit )
		SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].SetPoint = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Limit;
	if( SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out_SUM <= SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].LimitLow )
		SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].SetPoint = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].LimitLow;
	
	attitudeRatePID(&SYS_status->PID_Paras,&SYS_status->imu_fusion_module);// 角速度环PID 
	
	switch(pid_id)
	{
		case YAW:
			SYS_status->Manual_Status.MidServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].PIDcal_Out;
			;break;
		case PITCH:
			{
				SYS_status->Manual_Status.LeftServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].PIDcal_Out;
				SYS_status->Manual_Status.RightServo_Out_Sum -= SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].PIDcal_Out;
				
			}break;
		case ROLL:
			{
				SYS_status->Manual_Status.LeftServo_Out_Sum -= SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].PIDcal_Out;
				SYS_status->Manual_Status.RightServo_Out_Sum -= SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].PIDcal_Out;
			}break;
		case ALL:
			{
				SYS_status->Manual_Status.MidServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].PIDcal_Out;
				SYS_status->Manual_Status.LeftServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].PIDcal_Out 
															+ SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].PIDcal_Out;
				SYS_status->Manual_Status.RightServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].PIDcal_Out 
															- SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].PIDcal_Out;
			}break;
	}
	
	if(( SYS_status->Manual_Status.LeftServo_Out_Sum <= SYS_status->Manual_Status.LeftServo_Limit) && ( SYS_status->Manual_Status.LeftServo_Out_Sum >= SYS_status->Manual_Status.LeftServo_LimitLow))
		SYS_status->Manual_Status.LeftServo_Pulse = (u8)SYS_status->Manual_Status.LeftServo_Out_Sum;
	else
	{
		if( SYS_status->Manual_Status.LeftServo_Out_Sum > SYS_status->Manual_Status.LeftServo_Limit)
			SYS_status->Manual_Status.LeftServo_Pulse = SYS_status->Manual_Status.LeftServo_Limit;
		else
			SYS_status->Manual_Status.LeftServo_Pulse = SYS_status->Manual_Status.LeftServo_LimitLow;		
	}
	
	if(( SYS_status->Manual_Status.RightServo_Out_Sum <= SYS_status->Manual_Status.RightServo_Limit) && ( SYS_status->Manual_Status.RightServo_Out_Sum >= SYS_status->Manual_Status.RightServo_LimitLow))
		SYS_status->Manual_Status.RightServo_Pulse = (u8)SYS_status->Manual_Status.RightServo_Out_Sum;
	else
	{
		if( SYS_status->Manual_Status.RightServo_Out_Sum > SYS_status->Manual_Status.RightServo_Limit)
			SYS_status->Manual_Status.RightServo_Pulse = SYS_status->Manual_Status.RightServo_Limit;
		else
			SYS_status->Manual_Status.RightServo_Pulse = SYS_status->Manual_Status.RightServo_LimitLow;		
	}
	
	if(( SYS_status->Manual_Status.MidServo_Out_Sum <= SYS_status->Manual_Status.MidServo_Limit) && ( SYS_status->Manual_Status.MidServo_Out_Sum >= SYS_status->Manual_Status.MidServo_LimitLow))
		SYS_status->Manual_Status.MidServo_Pulse = (u8)SYS_status->Manual_Status.MidServo_Out_Sum;
	else
	{
		if( SYS_status->Manual_Status.MidServo_Out_Sum > SYS_status->Manual_Status.MidServo_Limit)
			SYS_status->Manual_Status.MidServo_Pulse = SYS_status->Manual_Status.MidServo_Limit;
		else
			SYS_status->Manual_Status.MidServo_Pulse = SYS_status->Manual_Status.MidServo_LimitLow;		
	}
/*	
u8 pid_id;
	pid_id = SYS_status->PID_Paras.PID_id;
	
	attitudeAnglePID(&SYS_status->PID_Paras,&SYS_status->imu_fusion_module);// 角度环PID
	switch(pid_id)
	{
		case YAW:
			SYS_status->Manual_Status.MidServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PIDcal_Out;
			;break;
		case PITCH:
			{
				SYS_status->Manual_Status.LeftServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out;
				SYS_status->Manual_Status.RightServo_Out_Sum -= SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out;
				
			}break;
		case ROLL:
			{
				SYS_status->Manual_Status.LeftServo_Out_Sum -= SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out;
				SYS_status->Manual_Status.RightServo_Out_Sum -= SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out;
			}break;
		case ALL:
			{
				SYS_status->Manual_Status.MidServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PIDcal_Out;
				SYS_status->Manual_Status.LeftServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out 
															+ SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out;
				SYS_status->Manual_Status.RightServo_Out_Sum += SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out 
															- SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out;
			}break;
	}
	
	if(( SYS_status->Manual_Status.LeftServo_Out_Sum <= SYS_status->Manual_Status.LeftServo_Limit) && ( SYS_status->Manual_Status.LeftServo_Out_Sum >= SYS_status->Manual_Status.LeftServo_LimitLow))
		SYS_status->Manual_Status.LeftServo_Pulse = (u8)SYS_status->Manual_Status.LeftServo_Out_Sum;
	else
	{
		if( SYS_status->Manual_Status.LeftServo_Out_Sum > SYS_status->Manual_Status.LeftServo_Limit)
			SYS_status->Manual_Status.LeftServo_Pulse = SYS_status->Manual_Status.LeftServo_Limit;
		else
			SYS_status->Manual_Status.LeftServo_Pulse = SYS_status->Manual_Status.LeftServo_LimitLow;		
	}
	
	if(( SYS_status->Manual_Status.RightServo_Out_Sum <= SYS_status->Manual_Status.RightServo_Limit) && ( SYS_status->Manual_Status.RightServo_Out_Sum >= SYS_status->Manual_Status.RightServo_LimitLow))
		SYS_status->Manual_Status.RightServo_Pulse = (u8)SYS_status->Manual_Status.RightServo_Out_Sum;
	else
	{
		if( SYS_status->Manual_Status.RightServo_Out_Sum > SYS_status->Manual_Status.RightServo_Limit)
			SYS_status->Manual_Status.RightServo_Pulse = SYS_status->Manual_Status.RightServo_Limit;
		else
			SYS_status->Manual_Status.RightServo_Pulse = SYS_status->Manual_Status.RightServo_LimitLow;		
	}
	
	if(( SYS_status->Manual_Status.MidServo_Out_Sum <= SYS_status->Manual_Status.MidServo_Limit) && ( SYS_status->Manual_Status.MidServo_Out_Sum >= SYS_status->Manual_Status.MidServo_LimitLow))
		SYS_status->Manual_Status.MidServo_Pulse = (u8)SYS_status->Manual_Status.MidServo_Out_Sum;
	else
	{
		if( SYS_status->Manual_Status.MidServo_Out_Sum > SYS_status->Manual_Status.MidServo_Limit)
			SYS_status->Manual_Status.MidServo_Pulse = SYS_status->Manual_Status.MidServo_Limit;
		else
			SYS_status->Manual_Status.MidServo_Pulse = SYS_status->Manual_Status.MidServo_LimitLow;		
	}*/
}

