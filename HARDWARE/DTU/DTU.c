#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stdlib.h"
#include "string.h"
#include "DTU.h"


void DTU_init(void)
{
	GPIO_InitTypeDef GPIO_def;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		//打开通用定时器4的时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);			//打开PB引脚的时钟
	
	GPIO_def.GPIO_Pin = GPIO_Pin_7;
	GPIO_def.GPIO_Mode = GPIO_Mode_IPD;							// 
//	GPIO_def.GPIO_OType = GPIO_OType_PP;						//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_def);
	

//数传模块接收机油门摇杆推满100，对应脉宽是0.586ms，推到负满量程-100，对应脉宽是1.414ms
	TIM_TimeBaseInitStruct.TIM_Prescaler = (9-1);				//72MHZ/9等于定时器计数器1秒钟计数的次数，也就是8MHZ，那每计数一次时间为0.000125ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM4_PERIOD;			//计数10000次,对应时间是1ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	///设置时钟分割:TDTS = Tck_tim


	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;				//设置为通道2
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;	//设置为下降沿检测
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;//TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			//Capture performed each time an edge is detected on the capture input
	TIM_ICInitStruct.TIM_ICFilter = 3<<4;						//Fsampling = Fck_int, N=8;
	
	TIM_ICInit(TIM4, &TIM_ICInitStruct);		

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//中断分组		
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;				//设置初始化的是TIM4的中断	
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;		//设置抢占优先级为2
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;				//设置响应优先级（子优先级）为0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				//使能定时器4这个中断
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);						//使能TIM4的IC2通道中断
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4, ENABLE);

}

void Command_manage(int32_t Command_length[])
{
	TIM_OCInitTypeDef TIM_OCInitStruct;
	uint8_t Control_Flag = 0;
	uint8_t FlyorClimb_Flag = 0;
	int32_t PWM_length = 0;

//判断当前哪个舵机受控
	if((Command_length[4]-4000) <= 7000)
		Control_Flag = PITCH;
	else 
		if( (Command_length[4]-4000) < 9000)
			Control_Flag = ROLL;
		else 
			Control_Flag = YAW;

//判断当前飞爬指令
	if((7800 < (Command_length[6]-4000)) && ((Command_length[6]-4000) < 8200))
		FlyorClimb_Flag = STOP;
	else 
		if((Command_length[6]-4000) < 7800)
			FlyorClimb_Flag = FLY;
		else 
			FlyorClimb_Flag = CLIMB;

		switch(FlyorClimb_Flag)
		{
			case FLY:
			{
			//爬行机构电机控制--停止爬行
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_Pulse = 150;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC4Init(TIM3, &TIM_OCInitStruct);
			//拍打机构电机控制--使能，占空比设置
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
				TIM_OCInitStruct.TIM_Pulse = ((abs(Command_length[1] - 12000)) >> 7)*80;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

				TIM_OC3Init(TIM3, &TIM_OCInitStruct);

			};break;
			case CLIMB:
			{
			//爬行机构电机控制--停止爬行
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_Pulse = 150;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
				TIM_OC3Init(TIM3, &TIM_OCInitStruct);

			//拍打机构电机控制--使能，占空比设置
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
				TIM_OCInitStruct.TIM_Pulse = ((abs(Command_length[1] - 12000)) >> 7)*80;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

				TIM_OC4Init(TIM3, &TIM_OCInitStruct);
			};break;
			default:
			{
			//爬行机构电机控制--停止爬行
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				//TIM_OCInitStruct.TIM_Pulse = 150;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC3Init(TIM3, &TIM_OCInitStruct);

			//爬行机构电机控制--停止爬行
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				//TIM_OCInitStruct.TIM_Pulse = 150;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC4Init(TIM3, &TIM_OCInitStruct);
			};break;
		}

		switch(Control_Flag)
		{
			case ROLL:
			{
			//俯仰--占空比设置
				PWM_length = ((Command_length[2] -12000)/75 + 75) << 1;
				TIM_SetCompare2(TIM2,PWM_length);
			};break;
			case PITCH:
			{
			//翻滚--占空比设置
				PWM_length = ((Command_length[2] -12000)/75 + 75) << 1;
				TIM_SetCompare3(TIM2,PWM_length);
			};break;
			case YAW:
			{
			//偏航--占空比设置
				PWM_length = (((Command_length[2] - 12000) >> 8) +72)<< 1;					
				TIM_SetCompare4(TIM2,PWM_length);
			};break;
			default:;break;
		}

}
