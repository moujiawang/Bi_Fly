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
	if (yawError > 180.0f) 
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	pid_paras->PID_YPR_para[YAW][RATE].SetPoint = PID_Update(&pid_paras->PID_YPR_para[YAW][ANGLE], yawError);
}
