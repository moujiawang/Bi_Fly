/* eeprom.c file
编写者：lisn3188
网址：www.chiplab7.net
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-05-05
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
将Flash用作EEPROM 用于保存偏置，标定数据，还有控制效果显著的PID值
------------------------------------
 */			  

#include "eeprom.h"


struct data_map Config;	//配置信息

void load_config(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	int16_t *temp_addr = (int16_t *)PAGE_Config;
	FLASH_Unlock();
	for(i=0 ; i< sizeof(Config)/2;i++){
		*ptr = *temp_addr;
		temp_addr++;
		ptr++;
	}
	FLASH_Lock();
	if(Config.is_good != (int16_t)0xA55A){ //数据无效 ，此时需要装载默认值。
		Config.is_good = 0xA55A;
		Config.dGx_offset = 0;
		Config.dGy_offset = 0;
		Config.dGz_offset = 0;
	
		Config.dMx_offset = 0;
		Config.dMy_offset = 0;
		Config.dMz_offset = 0;
	
		Config.dMx_scale =1.0f;
		Config.dMy_scale =1.0f;
		Config.dMz_scale =1.0f;
//ROLL
		Config.Kp_ROLL_RATE = 1.0f;					
		Config.Ki_ROLL_RATE = 1.0f;
		Config.Kd_ROLL_RATE = 1.0f;
		Config.Limit_ROLL_RATE = 1.0f;       		//output limit MAX
		Config.LimitLow_ROLL_RATE = 1.0f;    		//output limit MIN
		Config.Kp_ROLL_ANGLE = 1.0f;					
		Config.Ki_ROLL_ANGLE = 1.0f;
		Config.Kd_ROLL_ANGLE = 1.0f;
		Config.Limit_ROLL_ANGLE = 1.0f;       		//output limit MAX
		Config.LimitLow_ROLL_ANGLE = 1.0f;    		//output limit MIN
//YAW
		Config.Kp_YAW_RATE = 1.0f;					
		Config.Ki_YAW_RATE = 1.0f;
		Config.Kd_YAW_RATE = 1.0f;
		Config.Limit_YAW_RATE = 1.0f;       		//output limit MAX
		Config.LimitLow_YAW_RATE = 1.0f;    		//output limit MIN
		Config.Kp_YAW_ANGLE = 1.0f;					
		Config.Ki_YAW_ANGLE = 1.0f;
		Config.Kd_YAW_ANGLE = 1.0f;
		Config.Limit_YAW_ANGLE = 1.0f;       		//output limit MAX
		Config.LimitLow_YAW_ANGLE = 1.0f;    		//output limit MIN
//PITCH	
		Config.Kp_PITCH_RATE = 1.0f;	
		Config.Ki_PITCH_RATE = 1.0f;
		Config.Kd_PITCH_RATE = 1.0f;
		Config.Limit_PITCH_RATE = 1.0f;       		//output limit MAX
		Config.LimitLow_PITCH_RATE = 1.0f;    		//output limit MIN
		Config.Kp_PITCH_ANGLE = 1.0f;					
		Config.Ki_PITCH_ANGLE = 1.0f;
		Config.Kd_PITCH_ANGLE = 1.0f;
		Config.Limit_PITCH_ANGLE = 1.0f;       		//output limit MAX
		Config.LimitLow_PITCH_ANGLE = 1.0f;    		//output limit MIN
//舵机初始位置		
		Config.RightServo_Initpulse = 150;				
		Config.LeftServo_Initpulse = 150;
		Config.MidServo_Initpulse = 150;
//舵机限制位置
		Config.RightServo_Limit = 200;
		Config.RightServo_LimitLow = 100;
		Config.LeftServo_Limit = 200;
		Config.LeftServo_LimitLow = 100;	
		Config.MidServo_Limit = 200;
		Config.MidServo_LimitLow = 100;
		
		Config.rehresh_Hz = 25;						//refresh_Hz中存的值表示调参模式的运行频率，如果频率为x,则refresh_Hz中存放的数据为1000/x.
		
		
		Write_config();	 //将默认值写入flash
	}
}

//将当前配置写入flash
void Write_config(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	uint32_t ptemp_addr = PAGE_Config;
	FLASH_Unlock();
 	FLASH_ErasePage(PAGE_Config); //擦 页
	for(i=0;i<sizeof(Config)/2;i++){
	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
	 	ptemp_addr+=2;
	}
	FLASH_Lock();
}

void Refresh_flash_pid(PID_PARAS *pid_paras)
{
	switch(pid_paras->PID_id)
	{
		case YAW:
		{
			Config.Kp_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Kp;					
			Config.Ki_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Ki;	
			Config.Kd_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Kd;	
			Config.Limit_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Limit;	      			//output limit MAX
			Config.LimitLow_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].LimitLow;	    		//output limit MIN
			Config.Kp_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][ANGLE].Kp;						
			Config.Ki_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][ANGLE].Ki;	
			Config.Kd_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][ANGLE].Kd;	
			Config.Limit_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][RATE].Limit;	       			//output limit MAX
			Config.LimitLow_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][RATE].LimitLow;	    	//output limit MIN			
		};break;
		case PITCH:
		{
			Config.Kp_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Kp;					
			Config.Ki_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Ki;	
			Config.Kd_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Kd;	
			Config.Limit_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Limit;	       		//output limit MAX
			Config.LimitLow_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].LimitLow;	    	//output limit MIN
			Config.Kp_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][ANGLE].Kp;						
			Config.Ki_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][ANGLE].Ki;	
			Config.Kd_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][ANGLE].Kd;	
			Config.Limit_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][RATE].Limit;	       		//output limit MAX
			Config.LimitLow_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][RATE].LimitLow;	    //output limit MIN		
		}break;
		case ROLL:
		{
			Config.Kp_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Kp;					
			Config.Ki_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Ki;	
			Config.Kd_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Kd;	
			Config.Limit_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Limit;	       			//output limit MAX
			Config.LimitLow_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].LimitLow;	    	//output limit MIN
			Config.Kp_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][ANGLE].Kp;						
			Config.Ki_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][ANGLE].Ki;	
			Config.Kd_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][ANGLE].Kd;	
			Config.Limit_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][RATE].Limit;	       		//output limit MAX
			Config.LimitLow_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][RATE].LimitLow;	    	//output limit MIN	
		}break;
		case ALL:
		{
	//YAW
			Config.Kp_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Kp;					
			Config.Ki_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Ki;	
			Config.Kd_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Kd;	
			Config.Limit_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].Limit;	       		//output limit MAX
			Config.LimitLow_YAW_RATE = pid_paras->PID_YPR_para[YAW][RATE].LimitLow;	   		//output limit MIN
			Config.Kp_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][ANGLE].Kp;						
			Config.Ki_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][ANGLE].Ki;	
			Config.Kd_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][ANGLE].Kd;	
			Config.Limit_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][RATE].Limit;	      		//output limit MAX
			Config.LimitLow_YAW_ANGLE = pid_paras->PID_YPR_para[YAW][RATE].LimitLow;   		//output limit MIN		
	//PITCH
			Config.Kp_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Kp;					
			Config.Ki_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Ki;	
			Config.Kd_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Kd;	
			Config.Limit_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].Limit;	       		//output limit MAX
			Config.LimitLow_PITCH_RATE = pid_paras->PID_YPR_para[PITCH][RATE].LimitLow;	    	//output limit MIN
			Config.Kp_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][ANGLE].Kp;						
			Config.Ki_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][ANGLE].Ki;	
			Config.Kd_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][ANGLE].Kd;	
			Config.Limit_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][RATE].Limit;	       		//output limit MAX
			Config.LimitLow_PITCH_ANGLE = pid_paras->PID_YPR_para[PITCH][RATE].LimitLow;	    //output limit MIN		
	//ROLL	
			Config.Kp_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Kp;					
			Config.Ki_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Ki;	
			Config.Kd_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Kd;	
			Config.Limit_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].Limit;	       			//output limit MAX
			Config.LimitLow_ROLL_RATE = pid_paras->PID_YPR_para[ROLL][RATE].LimitLow;	    	//output limit MIN
			Config.Kp_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][ANGLE].Kp;						
			Config.Ki_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][ANGLE].Ki;	
			Config.Kd_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][ANGLE].Kd;	
			Config.Limit_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][RATE].Limit;	      		//output limit MAX
			Config.LimitLow_ROLL_ANGLE = pid_paras->PID_YPR_para[ROLL][RATE].LimitLow;	   		//output limit MIN	
		}break;
	}
	
	Write_config();
}

void Refresh_flash_pulse(MANUAL_STATUS *manual_status)
{
	Config.LeftServo_Initpulse = manual_status->LeftServo_Pulse;
	Config.RightServo_Initpulse = manual_status->RightServo_Pulse;
	Config.MidServo_Initpulse = manual_status->MidServo_Pulse;	
	Write_config();
}

void Sys_status_init(SYS_STATUS *sys_status)
{
	IMU_status_init(sys_status);
	
//DTU_NRF_Status
	sys_status->DTU_NRF_Status = 0x00;
//Flight_Status
	sys_status->Flight_Status.orientation = 0;
	sys_status->Flight_Status.x = 0;	
	sys_status->Flight_Status.y = 0;	
	sys_status->Flight_Status.z = 0;	
//拍打和爬行机构占空比初始化
	sys_status->Manual_Status.Fly_Pulse = 0;
	sys_status->Manual_Status.Climb_Pulse = 0;
//舵机初始化位置

/*	sys_status->Manual_Status.LeftServo_Pulse = 150;
	sys_status->Manual_Status.RightServo_Pulse = 150;	
	sys_status->Manual_Status.MidServo_Pulse = 150;*/
	sys_status->Manual_Status.LeftServo_Pulse = Config.LeftServo_Initpulse;
	sys_status->Manual_Status.RightServo_Pulse = Config.RightServo_Initpulse;	
	sys_status->Manual_Status.MidServo_Pulse = Config.MidServo_Initpulse;
//舵机限制位置
	sys_status->Manual_Status.RightServo_Limit = Config.RightServo_Limit ;
	sys_status->Manual_Status.RightServo_LimitLow = Config.RightServo_LimitLow;
	sys_status->Manual_Status.LeftServo_Limit = Config.LeftServo_Limit;
	sys_status->Manual_Status.LeftServo_LimitLow = Config.LeftServo_LimitLow;	
	sys_status->Manual_Status.MidServo_Limit = Config.MidServo_Limit;
	sys_status->Manual_Status.MidServo_LimitLow = Config.MidServo_LimitLow;	
//舵机PID计算值输出和
	sys_status->Manual_Status.RightServo_Out_Sum = 0;
	sys_status->Manual_Status.LeftServo_Out_Sum = 0;
	sys_status->Manual_Status.MidServo_Out_Sum = 0;
//IMU相关参数初始化	
	sys_status->imu_fusion_module.math_hz = 0;
	sys_status->imu_fusion_module.estimator.ex_inte = 0;
	sys_status->imu_fusion_module.estimator.ey_inte = 0;
	sys_status->imu_fusion_module.estimator.ez_inte = 0;
	sys_status->imu_fusion_module.estimator.q0 = 0;
	sys_status->imu_fusion_module.estimator.q1 = 0;
	sys_status->imu_fusion_module.estimator.q2 = 0;
	sys_status->imu_fusion_module.estimator.q3 = 0;	
	
	sys_status->imu_fusion_module.system_micrsecond = 0;
	sys_status->imu_fusion_module.upload_state = 0;
	sys_status->imu_fusion_module.ypr[YAW] = 0;
	sys_status->imu_fusion_module.ypr[PITCH] = 0;
	sys_status->imu_fusion_module.ypr[ROLL] = 0;
	sys_status->imu_fusion_module.ypr_rate[YAW] = 0;
	sys_status->imu_fusion_module.ypr_rate[PITCH] = 0;
	sys_status->imu_fusion_module.ypr_rate[ROLL] = 0;
	
	sys_status->PID_Paras.PID_id = 0;
	sys_status->PID_Paras.refresh_Hz = Config.rehresh_Hz;

}

void IMU_status_init(SYS_STATUS *sys_status)
{
//YAW
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Kp = Config.Kp_YAW_RATE;					
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Ki = Config.Ki_YAW_RATE;	
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Kd = Config.Kd_YAW_RATE;		
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Kp_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Ki_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Kd_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Kp_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Ki_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Kd_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Error = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].LastError = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].PreError = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].PIDcal_Out = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].PIDcal_Out_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].SetPoint = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].Limit = Config.Limit_YAW_RATE;      			//output limit MAX
	sys_status->PID_Paras.PID_YPR_para[YAW][RATE].LimitLow = Config.LimitLow_YAW_RATE;	   		//output limit MIN

	
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kp = Config.Kp_YAW_ANGLE;					
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Ki = Config.Ki_YAW_ANGLE;	
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kd = Config.Kd_YAW_ANGLE;	
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kp_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Ki_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kd_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kp_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Ki_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kd_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Error = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].LastError = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PreError = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PIDcal_Out = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].PIDcal_Out_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].SetPoint = 0;
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Limit = Config.Limit_YAW_ANGLE;	      		//output limit MAX
	sys_status->PID_Paras.PID_YPR_para[YAW][ANGLE].LimitLow = Config.LimitLow_YAW_ANGLE; 		//output limit MIN	
	
//PITCH
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kp = Config.Kp_PITCH_RATE;					
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Ki = Config.Ki_PITCH_RATE;	
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kd = Config.Kd_PITCH_RATE;	
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kp_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Ki_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kd_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kp_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Ki_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kd_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Error = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].LastError = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].PreError = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].PIDcal_Out = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].PIDcal_Out_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].SetPoint = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].Limit = Config.Limit_PITCH_RATE;      		//output limit MAX
	sys_status->PID_Paras.PID_YPR_para[PITCH][RATE].LimitLow = Config.LimitLow_PITCH_RATE;	   	//output limit MIN
	
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kp = Config.Kp_PITCH_ANGLE;					
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Ki = Config.Ki_PITCH_ANGLE;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kd = Config.Kd_PITCH_ANGLE;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kp_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Ki_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kd_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kp_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Ki_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kd_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Error = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].LastError = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PreError = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].PIDcal_Out_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].SetPoint = 0;
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Limit = Config.Limit_PITCH_ANGLE;	      	//output limit MAX
	sys_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].LimitLow = Config.LimitLow_PITCH_ANGLE; 	//output limit MIN	

//ROLL	
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kp = Config.Kp_ROLL_RATE;					
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Ki = Config.Ki_ROLL_RATE;	
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kd = Config.Kd_ROLL_RATE;	
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kp_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Ki_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kd_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kp_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Ki_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kd_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Error = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].LastError = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].PreError = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].PIDcal_Out = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].PIDcal_Out_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].SetPoint = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].Limit = Config.Limit_ROLL_RATE;	       		//output limit MAX
	sys_status->PID_Paras.PID_YPR_para[ROLL][RATE].LimitLow = Config.LimitLow_ROLL_RATE;  		//output limit MIN
	
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kp = Config.Kp_ROLL_ANGLE;						
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Ki = Config.Ki_ROLL_ANGLE;	
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kd = Config.Kd_ROLL_ANGLE;	
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kp_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Ki_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kd_OUT = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kp_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Ki_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kd_OUT_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Error = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].LastError = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PreError = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].PIDcal_Out_SUM = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].SetPoint = 0;
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Limit = Config.Limit_ROLL_ANGLE;	      	//output limit MAX
	sys_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].LimitLow = Config.LimitLow_ROLL_ANGLE; 		//output limit MIN
}


//------------------End of File----------------------------
