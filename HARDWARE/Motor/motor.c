#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "motor.h"

void motor_init(void)
{
	GPIO_InitTypeDef GPIO_def;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//��ͨ�ö�ʱ��2��ʱ��	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//��ͨ�ö�ʱ��3��ʱ��
	//�Ĵ����������ơ�����������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//��PA���ŵ�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		//��PA���ŵ�ʱ��	
	GPIO_def.GPIO_Pin = GPIO_Pin_7;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_def.GPIO_OType = GPIO_OType_PP;						//�������
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
			
	GPIO_Init(GPIOA,&GPIO_def);

			
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
		
	//���л���������ơ�����������
			
	GPIO_def.GPIO_Pin = GPIO_Pin_6;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_def.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_def.GPIO_OType = GPIO_OType_PP;						//�������
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIO_def);
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//��ͨ�ö�ʱ��3��ʱ��
			
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	//TIM3ʱ������
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/720���ڶ�ʱ��������1���Ӽ����Ĵ�����Ҳ����100000�Σ���ÿ����һ��ʱ��Ϊ0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 2200-1;					//����2200�Σ�Ҳ����22ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
			
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
/*********************************************************�������********************************************************/
	
	//TIM2ʱ������
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/720���ڶ�ʱ��������1���Ӽ����Ĵ�����Ҳ����100000�Σ���ÿ����һ��ʱ��Ϊ0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 2000-1;					//����2000�Σ�Ҳ����20ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
			
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	//Pitch������ơ����������ú�ʱ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//��PA���ŵ�ʱ��
		
	GPIO_def.GPIO_Pin = GPIO_Pin_1;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;						//�������
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_def.GPIO_PuPd = GPIO_PuPd_NOPULL;						//û����������
			
	GPIO_Init(GPIOA,&GPIO_def);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//��ͨ�ö�ʱ��2��ʱ��
			
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);		
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 150;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	//Roll������ơ����������ú�ʱ������
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//��PA���ŵ�ʱ��		
	GPIO_def.GPIO_Pin = GPIO_Pin_2;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;						//�������
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_def.GPIO_PuPd = GPIO_PuPd_NOPULL;						//û����������
			
	GPIO_Init(GPIOA,&GPIO_def);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//��ͨ�ö�ʱ��2��ʱ��
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 150;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	//Yaw������ơ����������ú�ʱ������
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//��PA���ŵ�ʱ��
		
	GPIO_def.GPIO_Pin = GPIO_Pin_3;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;						//�������
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_def.GPIO_PuPd = GPIO_PuPd_NOPULL;						//û����������
			
	GPIO_Init(GPIOA,&GPIO_def);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//��ͨ�ö�ʱ��2��ʱ��
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);		
	

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 150;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC4Init(TIM2, &TIM_OCInitStruct);	
	
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
}
