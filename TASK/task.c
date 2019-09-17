#include "nrf_protocol.h"
#include "attitude_pid.h"
#include "task.h"
#include "IMU.h"
#include "motor.h"
#include "24l01.h" 	



extern u8 Rx_buf[RX_PLOAD_WIDTH];
extern u8 Tx_buf[TX_PLOAD_WIDTH];





void Start_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	u8 tx_len = 0;
	static uint8_t offline_counter = 0;
	//////////////////IMU////////////////////
//	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	///////////////////////////////////////// 
	tx_len = Command_patch(Tx_buf, SYS_status,START_MODE);	 //打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf,tx_len);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status,rx_len);				//解包，分发数据
		Motor_action(&SYS_status->Manual_Status,START_TASK_DELAY);			//执行舵机指令操作，实现调整舵机初始位置的功能
		if(SYS_status->DTU_NRF_Status & WRITE_FLASH_FLAG)	//查看是否需要将数据写入flash
		{
			Config.LeftServo_Initpulse = SYS_status->Manual_Status.LeftServo_Pulse;
			Config.MidServo_Initpulse = SYS_status->Manual_Status.MidServo_Pulse;
			Config.RightServo_Initpulse = SYS_status->Manual_Status.RightServo_Pulse;
			Write_config();
			SYS_status->DTU_NRF_Status &= 0xbf;				//WRITE_FLASH_FLAG标志位清零
		}
		offline_counter = 0;
	}
	else													//接收应答信号失败
	{
		offline_counter++;
		if(offline_counter > 2*(1000/START_TASK_DELAY) )
		{
			SYS_status->DTU_NRF_Status &= NRF_DISCONNECTED;
			NRF24L01_FlushTX();
			NRF24L01_FlushRX();
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);		//清除TX_DS或MAX_RT中断标志
			SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
			Sys_status_init(SYS_status);						//初始化状态量SYS_Status
			Motor_action(&SYS_status->Manual_Status,START_TASK_DELAY);			//电机，舵机执行初始化操作
		}
	}	


}

void Manual_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	u8 tx_len = 0;
	static uint8_t offline_counter = 0;
	//////////////////IMU////////////////////
//	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	/////////////////////////////////////////
	tx_len = Command_patch(Tx_buf, SYS_status, MANUAL_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf,tx_len);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status,rx_len);				//解包，分发数据
		Motor_action(&SYS_status->Manual_Status,MANUAL_TASK_DELAY);			//根据解包后的指令，执行指令动作
		if(SYS_status->DTU_NRF_Status & WRITE_FLASH_FLAG)	//查看是否需要将数据写入flash
		{
/*			Config.LeftServo_Initpulse = SYS_status->Manual_Status.LeftServo_Pulse;
			Config.MidServo_Initpulse = SYS_status->Manual_Status.MidServo_Pulse;
			Config.RightServo_Initpulse = SYS_status->Manual_Status.RightServo_Pulse;*/
			Write_config();
			SYS_status->DTU_NRF_Status &= 0xbf;				//WRITE_FLASH_FLAG标志位清零
		}
		offline_counter = 0;
	}
	else													//接收应答信号失败
	{
		offline_counter++;
		if(offline_counter > 2*(1000/MANUAL_TASK_DELAY) )
		{
			SYS_status->DTU_NRF_Status &= NRF_DISCONNECTED;
			NRF24L01_FlushTX();
			NRF24L01_FlushRX();
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);		//清除TX_DS或MAX_RT中断标志
			SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
			Sys_status_init(SYS_status);						//初始化状态量SYS_Status
			Motor_action(&SYS_status->Manual_Status, MANUAL_TASK_DELAY);			//电机，舵机执行初始化操作
		}
	}	

}

void Flight_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	u8 tx_len = 0;
	static uint8_t offline_counter = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	/////////////////////////////////////////
	tx_len = Command_patch(Tx_buf, SYS_status, FLIGHT_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf, tx_len);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status,rx_len);				//解包，分发数据
		Flight_command(&SYS_status->Flight_Status);			//根据解包后的指令，执行指令动作
		if(SYS_status->DTU_NRF_Status & WRITE_FLASH_FLAG)	//查看是否需要将数据写入flash
		{
/*			Config.LeftServo_Initpulse = SYS_status->Manual_Status.LeftServo_Pulse;
			Config.MidServo_Initpulse = SYS_status->Manual_Status.MidServo_Pulse;
			Config.RightServo_Initpulse = SYS_status->Manual_Status.RightServo_Pulse;*/
			Write_config();
			SYS_status->DTU_NRF_Status &= 0xbf;				//WRITE_FLASH_FLAG标志位清零
		}
		offline_counter = 0;
	}
	else													//接收应答信号失败
	{
		offline_counter++;
		if(offline_counter > 2*(1000/FLIGHT_TASK_DELAY) )
		{
			SYS_status->DTU_NRF_Status &= NRF_DISCONNECTED;
			NRF24L01_FlushTX();
			NRF24L01_FlushRX();
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);		//清除TX_DS或MAX_RT中断标志
			SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
			Sys_status_init(SYS_status);						//初始化状态量SYS_Status
			Motor_action(&SYS_status->Manual_Status,FLIGHT_TASK_DELAY);			//电机，舵机执行初始化操作
		}
	}	

}

void Tuning_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	u8 tx_len = 0;
	static uint8_t offline_counter = 0;

	tx_len = Command_patch(Tx_buf, SYS_status, TUNING_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf,tx_len);

	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		offline_counter = 0;
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status,rx_len);			//解包，分发数据
		PID_command( SYS_status );							//根据解包后的指令，执行PID运算
		Motor_action(&SYS_status->Manual_Status, SYS_status->PID_Paras.refresh_Hz);		//执行电机操作
		if(SYS_status->DTU_NRF_Status & WRITE_FLASH_FLAG)	//查看是否需要将数据写入flash
		{
			switch(SYS_status->PID_Paras.PID_id)
			{
				case YAW:
				{
					Config.Kp_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kp;
					Config.Ki_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Ki;
					Config.Kd_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kd;
					Config.Kp_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Kp;
					Config.Ki_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Ki;
					Config.Kd_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Kd;
					
					Config.LimitLow_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].LimitLow;
					Config.Limit_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Limit;
					Config.LimitLow_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].LimitLow;
					Config.Limit_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Limit;
					Config.rehresh_Hz = SYS_status->PID_Paras.refresh_Hz;
				};break;
				case PITCH:
				{
					Config.Kp_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kp;
					Config.Ki_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Ki;
					Config.Kd_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kd;
					Config.Kp_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kp;
					Config.Ki_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Ki;
					Config.Kd_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kd;
					
					Config.LimitLow_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].LimitLow;
					Config.Limit_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Limit;
					Config.LimitLow_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].LimitLow;
					Config.Limit_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Limit;

					Config.rehresh_Hz = SYS_status->PID_Paras.refresh_Hz;					
				};break;
				case ROLL:
				{
					Config.Kp_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kp;
					Config.Ki_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Ki;
					Config.Kd_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kd;
					Config.Kp_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kp;
					Config.Ki_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Ki;
					Config.Kd_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kd;

					Config.LimitLow_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].LimitLow;
					Config.Limit_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Limit;
					Config.LimitLow_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].LimitLow;
					Config.Limit_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Limit;
					Config.rehresh_Hz = SYS_status->PID_Paras.refresh_Hz;
				};break;
				case ALL:
				{
					//PITCH
					Config.Kp_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kp;
					Config.Ki_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Ki;
					Config.Kd_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Kd;
					Config.Kp_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kp;
					Config.Ki_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Ki;
					Config.Kd_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Kd;
					Config.LimitLow_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].LimitLow;
					Config.Limit_PITCH_ANGLE = SYS_status->PID_Paras.PID_YPR_para[PITCH][ANGLE].Limit;
					Config.LimitLow_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].LimitLow;
					Config.Limit_PITCH_RATE = SYS_status->PID_Paras.PID_YPR_para[PITCH][RATE].Limit;	
					//YAW
					Config.Kp_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kp;
					Config.Ki_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Ki;
					Config.Kd_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Kd;
					Config.Kp_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Kp;
					Config.Ki_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Ki;
					Config.Kd_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Kd;
					Config.LimitLow_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].LimitLow;
					Config.Limit_YAW_ANGLE = SYS_status->PID_Paras.PID_YPR_para[YAW][ANGLE].Limit;
					Config.LimitLow_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].LimitLow;
					Config.Limit_YAW_RATE = SYS_status->PID_Paras.PID_YPR_para[YAW][RATE].Limit;
					//ROLL
					Config.Kp_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kp;
					Config.Ki_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Ki;
					Config.Kd_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Kd;
					Config.Kp_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kp;
					Config.Ki_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Ki;
					Config.Kd_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Kd;
					Config.LimitLow_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].LimitLow;
					Config.Limit_ROLL_ANGLE = SYS_status->PID_Paras.PID_YPR_para[ROLL][ANGLE].Limit;
					Config.LimitLow_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].LimitLow;
					Config.Limit_ROLL_RATE = SYS_status->PID_Paras.PID_YPR_para[ROLL][RATE].Limit;
					Config.rehresh_Hz = SYS_status->PID_Paras.refresh_Hz;
				};break;

			}		
//舵机输出限制
			Config.RightServo_Limit = SYS_status->Manual_Status.RightServo_Limit;
			Config.RightServo_LimitLow = SYS_status->Manual_Status.RightServo_LimitLow;
			Config.LeftServo_Limit = SYS_status->Manual_Status.LeftServo_Limit;
			Config.LeftServo_LimitLow = SYS_status->Manual_Status.LeftServo_LimitLow;	
			Config.MidServo_Limit = SYS_status->Manual_Status.MidServo_Limit;
			Config.MidServo_LimitLow = SYS_status->Manual_Status.MidServo_LimitLow;	
			
			Write_config();
			SYS_status->DTU_NRF_Status &= 0xbf;				//WRITE_FLASH_FLAG标志位清零
		}
	}
	else													//接收应答信号失败
	{
		offline_counter++;
		if(offline_counter > 2*(1000/TUNING_TASK_DELAY) )
		{
			SYS_status->DTU_NRF_Status &= NRF_DISCONNECTED;
			NRF24L01_FlushTX();
			NRF24L01_FlushRX();
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);		//清除TX_DS或MAX_RT中断标志
			SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
			Sys_status_init(SYS_status);						//初始化状态量SYS_Status
			Motor_action(&SYS_status->Manual_Status, SYS_status->PID_Paras.refresh_Hz);			//电机，舵机执行初始化操作
		}
	}
}

void Fault_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	u8 tx_len = 0;
	static uint8_t offline_counter = 0;
	if(NRF24L01_Check())
		SYS_status->DTU_NRF_Status &= NRF_OFF;
	else
	{
		SYS_status->DTU_NRF_Status |= NRF_ON;
		tx_len = Command_patch(Tx_buf, SYS_status, FAULT_MODE);			//打包数据，更新Tx_buf，准备发送	
		rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf,tx_len);
		if((rx_len>0)&&(rx_len<33))
		{
			SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
			offline_counter = 0;
			Command_dispatch(Rx_buf,SYS_status,rx_len);				//解包，分发数据
		}
		else
		{
			offline_counter++;
			if(offline_counter > 2*(1000/FAULT_TASK_DELAY) )
			{
				SYS_status->DTU_NRF_Status &= NRF_DISCONNECTED;
				NRF24L01_FlushTX();
				NRF24L01_FlushRX();
				NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,RX_OK|TX_OK|MAX_TX);		//清除TX_DS或MAX_RT中断标志
			}

		}
	}
}




