#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "motor.h"

void motor_init(void)
{
	GPIO_InitTypeDef GPIO_def;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//打开通用定时器2的时钟	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//打开通用定时器3的时钟
	//拍打机构电机控制——引脚配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//打开PA引脚的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		//打开PA引脚的时钟	
	GPIO_def.GPIO_Pin = GPIO_Pin_7;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_def.GPIO_OType = GPIO_OType_PP;						//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
			
	GPIO_Init(GPIOA,&GPIO_def);

			
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
		
	//爬行机构电机控制——引脚配置
			
	GPIO_def.GPIO_Pin = GPIO_Pin_6;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_def.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_def.GPIO_OType = GPIO_OType_PP;						//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIO_def);
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//打开通用定时器3的时钟
			
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	//TIM3时钟配置
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/720等于定时器计数器1秒钟计数的次数，也就是100000次，那每计数一次时间为0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 2200-1;					//计数2200次，也就是22ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
			
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
/*********************************************************舵机控制********************************************************/
	
	//TIM2时钟配置
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/720等于定时器计数器1秒钟计数的次数，也就是100000次，那每计数一次时间为0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 2000-1;					//计数2000次，也就是20ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
			
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	//Pitch舵机控制——引脚配置和时钟配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//打开PA引脚的时钟
		
	GPIO_def.GPIO_Pin = GPIO_Pin_1;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;						//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_def.GPIO_PuPd = GPIO_PuPd_NOPULL;						//没有上拉下拉
			
	GPIO_Init(GPIOA,&GPIO_def);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//打开通用定时器2的时钟
			
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);		
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 150;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	//Roll舵机控制——引脚配置和时钟配置
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//打开PA引脚的时钟		
	GPIO_def.GPIO_Pin = GPIO_Pin_2;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;						//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_def.GPIO_PuPd = GPIO_PuPd_NOPULL;						//没有上拉下拉
			
	GPIO_Init(GPIOA,&GPIO_def);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//打开通用定时器2的时钟
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 150;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	//Yaw舵机控制——引脚配置和时钟配置
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//打开PA引脚的时钟
		
	GPIO_def.GPIO_Pin = GPIO_Pin_3;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;						//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_def.GPIO_PuPd = GPIO_PuPd_NOPULL;						//没有上拉下拉
			
	GPIO_Init(GPIOA,&GPIO_def);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//打开通用定时器2的时钟
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
