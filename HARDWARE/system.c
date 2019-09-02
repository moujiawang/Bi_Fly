#include "system.h"

extern SYS_STATUS SYS_Status;
void System_init(void)
{

	delay_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);									//�жϷ���	
//	DTU_init();																		//����ģ���ʼ��

	load_config();  																//��flash�ж�ȡ������Ϣ -->eeprom.c
	Sys_status_init(&SYS_Status);													//��ʼ��״̬��SYS_Status
	motor_init(&SYS_Status.Manual_Status);											//������ƶ�ʱ����ʼ��
	TIM_Cmd(TIM2, ENABLE);															//ʹ��TIM2
	TIM_Cmd(TIM3, ENABLE);															//ʹ��TIM3
	NRF24L01_Init(TX_MODE);    														//��ʼ��NRF24L01Ϊ����ģʽ    
	Initial_UART1(115200L);															//��ʼ������
	////////////////IMU/////////////////
	imu_fusion_init(&SYS_Status.imu_fusion_module);
	////////////////////////////////////


	SYS_Status.DTU_NRF_Status |= FAULT_MODE;				 						//ģʽ��Ϣ����Ϊ START MODE

			
}



