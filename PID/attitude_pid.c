#include "attitude_pid.h"
#include "IncPID.h"



/**************************************************************/
//��������attitudeRatePID ���ٶ�PID��
//��  �룺PID_PARAS��PID�Ĳ����� , error
//��  �����ǶȻ�PID�������������ֱ�ӵõ����ƶ�����ռ�ձȵ�ֵ
//˵  �����˳�������ɽ��ٶȻ�PID����ֵ
/**************************************************************/
void attitudeRatePID(PID_PARAS * pid_paras, IMUFusion * imu_data)	/* ���ٶȻ�PID */
{
	PID_Update(&pid_paras->PID_YPR_para[YAW][RATE], pid_paras->PID_YPR_para[YAW][RATE].SetPoint - imu_data->ypr_rate[YAW]);
	PID_Update(&pid_paras->PID_YPR_para[PITCH][RATE], pid_paras->PID_YPR_para[PITCH][RATE].SetPoint - imu_data->ypr_rate[PITCH]);
	PID_Update(&pid_paras->PID_YPR_para[ROLL][RATE], pid_paras->PID_YPR_para[ROLL][RATE].SetPoint - imu_data->ypr_rate[ROLL]);
}

/**************************************************************/
//��������attitudeRatePID �Ƕ�PID��
//��  �룺PID_PARAS��PID�Ĳ����� , error
//��  �����ǻ�PID����������õ������Ϊ����Ľ��ٶ�ֵ��Ϊ����Ľ��ٶȻ��ṩ����
//˵  �����˳�������ɽǶȻ�PID����ֵ
/**************************************************************/
void attitudeAnglePID(PID_PARAS * pid_paras, IMUFusion * imu_data)	/* �ǶȻ�PID */
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
	
	attitudeAnglePID(&SYS_status->PID_Paras,&SYS_status->imu_fusion_module);// �ǶȻ�PID
	
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
	
	attitudeRatePID(&SYS_status->PID_Paras,&SYS_status->imu_fusion_module);// ���ٶȻ�PID 
	
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
	
	attitudeAnglePID(&SYS_status->PID_Paras,&SYS_status->imu_fusion_module);// �ǶȻ�PID
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

