#include "IncPID.h"
#include "nrf_protocol.h"


void IncPID_Init(PID_PARAS *PID_paras) 
{	
	u8 i = 0;
	for(i = 0; i<3; i++)
	{
		PID_paras->PID_YPR_para[i].Error_Int = 0;            	//e[k]
		PID_paras->PID_YPR_para[i].LastError_Int = 0;            //e[k-1]
		PID_paras->PID_YPR_para[i].PreError_Int = 0;             //e[k-2]
		PID_paras->PID_YPR_para[i].Error_Ext = 0;            	//e[k]
		PID_paras->PID_YPR_para[i].LastError_Ext = 0;            //e[k-1]
		PID_paras->PID_YPR_para[i].PreError_Ext = 0;             //e[k-2]
		PID_paras->PID_YPR_para[i].Kp_Int=0;      			//比例常数 Proportional Const
		PID_paras->PID_YPR_para[i].Ki_Int=0;        		//积分常数  Integral Const
		PID_paras->PID_YPR_para[i].Kd_Int=0;      			//微分常数 Derivative Const
		PID_paras->PID_YPR_para[i].Kp_Ext=0;      			//比例常数 Proportional Const
		PID_paras->PID_YPR_para[i].Ki_Ext=0;        		//积分常数  Integral Const
		PID_paras->PID_YPR_para[i].Kd_Ext=0;      			//微分常数 Derivative Const
	}
}

/**************************************************************/
//函数名：IncPID_Cal 增量式PID计算
//输  入：
//输  出：
//说  明：
/**************************************************************/
uint16_t IncPID_Cal(PID_PARA *PID_para, IMUFusion *Attitude, YPR_ID YPR_id)  
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
}
