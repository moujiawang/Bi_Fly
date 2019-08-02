#include "nrf_protocol.h"
#include "task.h"
#include "IMU.h"
#include "motor.h"
#include "24l01.h" 	



extern u8 Rx_buf[RX_PLOAD_WIDTH];
extern u8 Tx_buf[TX_PLOAD_WIDTH];





void Start_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;

	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	///////////////////////////////////////// 
	Command_patch(Tx_buf, SYS_status,START_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//解包，分发数据
	}
	else													//接收应答信号失败
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRF模块与主控芯片没有连接			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRF模块与主控芯片连接正常	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	


}

void Manual_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	/////////////////////////////////////////
	Command_patch(Tx_buf, SYS_status, MANUAL_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//解包，分发数据
		Manual_command(&SYS_status->Manual_Status);			//根据解包后的指令，执行指令动作
	}
	else													//接收应答信号失败
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRF模块与主控芯片没有连接			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRF模块与主控芯片连接正常	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	

}

void Flight_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	/////////////////////////////////////////
	Command_patch(Tx_buf, SYS_status, FLIGHT_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//解包，分发数据
		Flight_command(&SYS_status->Flight_Status);			//根据解包后的指令，执行指令动作
	}
	else													//接收应答信号失败
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRF模块与主控芯片没有连接			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRF模块与主控芯片连接正常	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	

}

void Tuning_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//更新了角速度，角加速度
	/////////////////////////////////////////
	Command_patch(Tx_buf, SYS_status, TUNING_MODE);	 		//打包数据，更新Tx_buf，准备发送
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//接收应答信号成功
	{	
		//刷新状态标志和模式信息
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//解包，分发数据
		PID_command( SYS_status );			//根据解包后的指令，执行指令动作
	}
	else													//接收应答信号失败
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRF模块与主控芯片没有连接			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRF模块与主控芯片连接正常	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	
}

void Fault_task(SYS_STATUS *SYS_status)
{
	
}

void UpdateIMU_task(SYS_STATUS *SYS_status)
{
							
}