#include "IncPID.h"
#include "nrf_protocol.h"


void IncPID_Init(PID_PARAS *PID_paras) 
{	
	u8 i = 0;
	for(i = 0; i<3; i++)
	{
		PID_paras->PID_YPR_para[i][RATE].Error = 0;
		PID_paras->PID_YPR_para[i][RATE].LastError = 0;
		PID_paras->PID_YPR_para[i][RATE].PreError = 0;
		PID_paras->PID_YPR_para[i][RATE].Kd_OUT = 0;
		PID_paras->PID_YPR_para[i][RATE].Ki_OUT = 0;
		PID_paras->PID_YPR_para[i][RATE].Kp_OUT = 0;
		PID_paras->PID_YPR_para[i][RATE].PIDcal_Out = 0;
		PID_paras->PID_YPR_para[i][RATE].SetPoint = 0;
		PID_paras->PID_YPR_para[i][RATE].Kd = 0;
		PID_paras->PID_YPR_para[i][RATE].Ki = 0;
		PID_paras->PID_YPR_para[i][RATE].Kp = 0;
		PID_paras->PID_YPR_para[i][RATE].Limit = 0;
		PID_paras->PID_YPR_para[i][RATE].LimitLow = 0;
		
		PID_paras->PID_YPR_para[i][ANGLE].Error = 0;
		PID_paras->PID_YPR_para[i][ANGLE].LastError = 0;
		PID_paras->PID_YPR_para[i][ANGLE].PreError = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Kd_OUT = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Ki_OUT = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Kp_OUT = 0;
		PID_paras->PID_YPR_para[i][ANGLE].PIDcal_Out = 0;
		PID_paras->PID_YPR_para[i][ANGLE].SetPoint = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Kd = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Ki = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Kp = 0;
		PID_paras->PID_YPR_para[i][ANGLE].Limit = 0;
		PID_paras->PID_YPR_para[i][ANGLE].LimitLow = 0;
	}
}

/**************************************************************/
//函数名：PID_update 增量式PID计算
//输  入：PID_PARA , error
//输  出：PID计算输出
//说  明：此程序是完成单环PID计算值（已经完成了PID值的最小最大化处理）
/**************************************************************/
float PID_Update(PID_PARA* pid, const float error)
{
	float output;

	pid->Error = error;   

	pid->Kp_OUT = pid->Kp * ( pid->Error - pid->LastError );
	pid->Ki_OUT = pid->Ki * pid->Error;
	pid->Kd_OUT = pid->Kd * (pid->Error + pid->PreError - 2 * pid->LastError);

	output = pid->Kp_OUT + pid->Ki_OUT + pid->Kd_OUT;

	//更新PID的误差数据
	pid->PreError = pid->LastError;
	pid->LastError = pid->Error;
	if((output < pid->Limit) && (output >= pid->LimitLow))
		pid->PIDcal_Out = output;
	else
		if(output >= pid->Limit)
			pid->PIDcal_Out = pid->Limit;
		else 
			pid->PIDcal_Out = pid->LimitLow;
	
	return pid->PIDcal_Out;
}


/**************************************************************/
//函数名：PID_Reset PID复位
//输  入：
//输  出：
//说  明：
/**************************************************************/
void PID_Reset(PID_PARAS* PID_paras)
{
	u8 i = 0;
	u8 pid_id = PID_paras->PID_id;
	if(pid_id  == ALL)
	{
		for(i = 0; i < 3; i++)
		{
			PID_paras->PID_YPR_para[i][RATE].Error = 0;
			PID_paras->PID_YPR_para[i][RATE].LastError = 0;
			PID_paras->PID_YPR_para[i][RATE].PreError = 0;
			PID_paras->PID_YPR_para[i][RATE].Kd_OUT = 0;
			PID_paras->PID_YPR_para[i][RATE].Ki_OUT = 0;
			PID_paras->PID_YPR_para[i][RATE].Kp_OUT = 0;
			PID_paras->PID_YPR_para[i][RATE].PIDcal_Out = 0;
			PID_paras->PID_YPR_para[i][RATE].SetPoint = 0;
			PID_paras->PID_YPR_para[i][ANGLE].Error = 0;
			PID_paras->PID_YPR_para[i][ANGLE].LastError = 0;
			PID_paras->PID_YPR_para[i][ANGLE].PreError = 0;
			PID_paras->PID_YPR_para[i][ANGLE].Kd_OUT = 0;
			PID_paras->PID_YPR_para[i][ANGLE].Ki_OUT = 0;
			PID_paras->PID_YPR_para[i][ANGLE].Kp_OUT = 0;
			PID_paras->PID_YPR_para[i][ANGLE].PIDcal_Out = 0;
			PID_paras->PID_YPR_para[i][ANGLE].SetPoint = 0;
		}
	}
	else
	{
		PID_paras->PID_YPR_para[pid_id][RATE].Error = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].LastError = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].PreError = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].Kd_OUT = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].Ki_OUT = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].Kp_OUT = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].PIDcal_Out = 0;
		PID_paras->PID_YPR_para[pid_id][RATE].SetPoint = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].Error = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].LastError = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].PreError = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].Kd_OUT = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].Ki_OUT = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].Kp_OUT = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].PIDcal_Out = 0;
		PID_paras->PID_YPR_para[pid_id][ANGLE].SetPoint = 0;
	}

}

/**************************************************************/
//函数名：IncPID_Cal 增量式PID计算
//输  入：
//输  出：
//说  明：
/**************************************************************/
/*uint16_t IncPID_Cal(PID_PARA *PID_para, IMUFusion *Attitude, YPR_ID YPR_id)  
{
	PID_para->Error_Ext = PID_para->SetPoint_Ext  - Attitude->ypr[YPR_id];  													//增量计算
	PID_para->PIDcal_ExtOut = ((PID_para->Kp_Ext * ( PID_para->Error_Ext - PID_para->LastError_Ext ))                			//比例环节
                            +(PID_para->Ki_Ext * PID_para->Error_Ext)     														//积分环节	
							+(PID_para->Kd_Ext * (PID_para->Error_Ext + PID_para->PreError_Ext - 2 * PID_para->LastError_Ext)));//微分环节

	PID_para->PreError_Ext = PID_para->LastError_Ext;                    														//存储误差，用于下次计算
	PID_para->LastError_Ext = PID_para->Error_Ext;
	
	PID_para->SetPoint_Int = PID_para->PIDcal_ExtOut;
	PID_para->Error_Int = PID_para->SetPoint_Int -Attitude->ypr_rate[YPR_id];  
	PID_para->PIDcal_IntOut = ((PID_para->Kp_Int * ( PID_para->Error_Int - PID_para->LastError_Int))                			 //比例环节
                            +(PID_para->Ki_Int * PID_para->Error_Int)     														 //积分环节	
							+(PID_para->Kd_Int * (PID_para->Error_Int + PID_para->PreError_Int - 2 * PID_para->LastError_Int))); //微分环节
              
	PID_para->PreError_Int = PID_para->LastError_Int;                        													//存储误差，用于下次计算
	PID_para->LastError_Int = PID_para->Error_Int; 
	
	return(PID_para->PIDcal_IntOut);                                    														 //返回增量值
}*/
