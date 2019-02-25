#include "IncPID.h"


void IncPID_Init(PID_PARA *PID_Yaw_para, PID_PARA *PID_Pitch_para, PID_PARA *PID_Roll_para) 
{
    PID_Yaw_para->Error_Int = 0;            	//e[k]
    PID_Yaw_para->LastError_Int = 0;            //e[k-1]
	PID_Yaw_para->PreError_Int = 0;             //e[k-2]
	PID_Yaw_para->Error_Ext = 0;            	//e[k]
    PID_Yaw_para->LastError_Ext = 0;            //e[k-1]
	PID_Yaw_para->PreError_Ext = 0;             //e[k-2]
    PID_Yaw_para->Kp_Int=0;      			//�������� Proportional Const
    PID_Yaw_para->Ki_Int=0;        		//���ֳ���  Integral Const
    PID_Yaw_para->Kd_Int=0;      			//΢�ֳ��� Derivative Const
	PID_Yaw_para->Kp_Ext=0;      			//�������� Proportional Const
    PID_Yaw_para->Ki_Ext=0;        		//���ֳ���  Integral Const
    PID_Yaw_para->Kd_Ext=0;      			//΢�ֳ��� Derivative Const

	PID_Pitch_para->Error_Int = 0;            	//e[k]
    PID_Pitch_para->LastError_Int = 0;          //e[k-1]
	PID_Pitch_para->PreError_Int = 0;           //e[k-2]
	PID_Pitch_para->Error_Ext = 0;            	//e[k]
    PID_Pitch_para->LastError_Ext = 0;          //e[k-1]
	PID_Pitch_para->PreError_Ext = 0;           //e[k-2]
    PID_Pitch_para->Kp_Int=0;      		//�������� Proportional Const
    PID_Pitch_para->Ki_Int=0;        		//���ֳ���  Integral Const
    PID_Pitch_para->Kd_Int=0;      		//΢�ֳ��� Derivative Const
	PID_Pitch_para->Kp_Ext=0;      		//�������� Proportional Const
    PID_Pitch_para->Ki_Ext=0;        		//���ֳ���  Integral Const
    PID_Pitch_para->Kd_Ext=0;      		//΢�ֳ��� Derivative Const
	
	PID_Roll_para->Error_Int = 0;            	//e[k]
    PID_Roll_para->LastError_Int = 0;           //e[k-1]
	PID_Roll_para->PreError_Int = 0;            //e[k-2]
	PID_Roll_para->Error_Ext = 0;            	//e[k]
    PID_Roll_para->LastError_Ext = 0;           //e[k-1]
	PID_Roll_para->PreError_Ext = 0;            //e[k-2]
    PID_Roll_para->Kp_Int=0;      			//�������� Proportional Const
    PID_Roll_para->Ki_Int=0;        		//���ֳ���  Integral Const
    PID_Roll_para->Kd_Int=0;      			//΢�ֳ��� Derivative Const
	PID_Roll_para->Kp_Ext=0;      			//�������� Proportional Const
    PID_Roll_para->Ki_Ext=0;        		//���ֳ���  Integral Const
    PID_Roll_para->Kd_Ext=0;      			//΢�ֳ��� Derivative Const
}

/**************************************************************/
//��������IncPID_Cal ����ʽPID����
//��  �룺
//��  ����
//˵  ����
/**************************************************************/
uint16_t IncPID_Cal(PID_PARA *PID_para, float Angle_ActualVal, float rad_ActualVal)
{
	PID_para->Error_Int = PID_para->SetPoint_Ext - Angle_ActualVal;  															//��������
	PID_para->PIDcal_ExtOut = ((PID_para->Kp_Ext * ( PID_para->Error_Ext - PID_para->LastError_Ext ))                			//��������
                            +(PID_para->Ki_Ext * PID_para->Error_Ext)     													//���ֻ���	
							+(PID_para->Kd_Ext * (PID_para->Error_Ext + PID_para->PreError_Ext - 2 * PID_para->LastError_Ext)));//΢�ֻ���

	PID_para->PreError_Ext = PID_para->LastError_Ext;                    															//�洢�������´μ���
	PID_para->LastError_Ext = PID_para->Error_Ext;
	
	PID_para->SetPoint_Int = PID_para->PIDcal_ExtOut;
	PID_para->Error_Int = PID_para->SetPoint_Int -rad_ActualVal;
	PID_para->PIDcal_IntOut = ((PID_para->Kp_Int * ( PID_para->Error_Int - PID_para->LastError_Int))                				//��������
                            +(PID_para->Ki_Int * PID_para->Error_Int)     													//���ֻ���	
							+(PID_para->Kd_Int * (PID_para->Error_Int + PID_para->PreError_Int - 2 * PID_para->LastError_Int))); 	//΢�ֻ���
              
	PID_para->PreError_Int = PID_para->LastError_Int;                    															//�洢�������´μ���
	PID_para->LastError_Int = PID_para->Error_Int;
	
	return(PID_para->PIDcal_IntOut);                                    														//��������ֵ
}