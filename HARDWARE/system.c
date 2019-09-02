#include "system.h"

extern SYS_STATUS SYS_Status;
void System_init(void)
{

	delay_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);									//中断分组	
//	DTU_init();																		//数传模块初始化

	load_config();  																//从flash中读取配置信息 -->eeprom.c
	Sys_status_init(&SYS_Status);													//初始化状态量SYS_Status
	motor_init(&SYS_Status.Manual_Status);											//电机控制定时器初始化
	TIM_Cmd(TIM2, ENABLE);															//使能TIM2
	TIM_Cmd(TIM3, ENABLE);															//使能TIM3
	NRF24L01_Init(TX_MODE);    														//初始化NRF24L01为发送模式    
	Initial_UART1(115200L);															//初始化串口
	////////////////IMU/////////////////
	imu_fusion_init(&SYS_Status.imu_fusion_module);
	////////////////////////////////////


	SYS_Status.DTU_NRF_Status |= FAULT_MODE;				 						//模式信息更新为 START MODE

			
}



