#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "nrf_protocol.h"
#include "stdlib.h"
#include "string.h"
#include "IncPID.h"
#include "motor.h"
#include "attitude_pid.h"
#include "Data_map.h"



void motor_init(MANUAL_STATUS *manual_status)
{
	GPIO_InitTypeDef GPIO_def;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//打开通用定时器2的时钟	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//打开通用定时器3的时钟
	//拍打机构电机控制——引脚配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//打开PA引脚的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//打开PB引脚的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		//打开AFIO的时钟	
	
	GPIO_def.GPIO_Pin = GPIO_Pin_0;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
			
	GPIO_Init(GPIOB,&GPIO_def);
		
	//爬行机构电机控制——引脚配置
			
	GPIO_def.GPIO_Pin = GPIO_Pin_1;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;					
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB,&GPIO_def);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		//打开通用定时器3的时钟
			
	//TIM3时钟配置
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);				//72MHZ/720等于定时器计数器1秒钟计数的次数，也就是100000次，那每计数一次时间为0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 100;					//计数101次，也就是1.01ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
			
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	//爬行机构电机比较输出模式配置
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
//	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	//拍打机构电机控制--不使能，占空比设置
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
//	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	
/*********************************************************舵机控制********************************************************/
	
	//TIM2时钟配置
	TIM_TimeBaseInitStruct.TIM_Prescaler = (720-1);					//72MHZ/720等于定时器计数器1秒钟计数的次数，也就是100000次，那每计数一次时间为0.01ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 350-1;						//计数500次，也就是5ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
			
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	//left舵机控制——引脚配置和时钟配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//打开PA引脚的时钟
		
	GPIO_def.GPIO_Pin = GPIO_Pin_1;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;							//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIO_def);

	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = manual_status->LeftServo_Pulse;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	//mid舵机控制——引脚配置和时钟配置		
	GPIO_def.GPIO_Pin = GPIO_Pin_2;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;							//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIO_def);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//打开通用定时器2的时钟
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = manual_status->MidServo_Pulse;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	//right舵机控制——引脚配置和时钟配置		
	GPIO_def.GPIO_Pin = GPIO_Pin_3;
	GPIO_def.GPIO_Mode = GPIO_Mode_AF_PP;							//推免输出
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIO_def);


	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = manual_status->RightServo_Pulse;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	TIM_OC4Init(TIM2, &TIM_OCInitStruct);	
	
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
}

/*===========================================================================
* 函数 ：Command_manage                                  
* 输入 ：int32_t Command_length[],MANUAL_STATUS* Manual_Status
* 返回 ：无；		  				
* 说明 ：此函数是用于解包DTU数据包，解析遥控的指令，用于控制飞爬的电机速度控制
		和三个舵机位置控制
============================================================================*/
void Command_manage(int32_t Command_length[],MANUAL_STATUS* Manual_Status)
{
	TIM_OCInitTypeDef TIM_OCInitStruct;

	uint16_t Control_Pulse = 0;
	uint16_t Motion_Pulse = 0;
	
//判断当前飞爬指令
	if((7800 < (Command_length[6]-4000)) && ((Command_length[6]-4000) < 8200))
		Manual_Status->Fly_or_Climb_Status = STOP;
	else 
		if((Command_length[6]-4000) < 7800)
			Manual_Status->Fly_or_Climb_Status = FLY;
		else 
			Manual_Status->Fly_or_Climb_Status = CLIMB;
//飞爬速度指令处理
	if( (Command_length[1] <= 12064)  && (Command_length[1] > 11936) )
		Motion_Pulse = 0;
	else
		if( (Command_length[1] > 12064) && (Command_length[1]<= 15264) )
			Motion_Pulse =(( Command_length[1] - 12064) >> 7) * 4;
		else
			if((Command_length[1] > 8736) && (Command_length[1] <= 11936))
				Motion_Pulse =(( 11936 - Command_length[1]) >> 7) * 4;
			else
				if((Command_length[1] > 8000) && (Command_length[1] < 16000))
					Motion_Pulse = 100;
				else 
					Motion_Pulse = 0;

	Control_Pulse = ((Command_length[2] -12000)/75 + 75) << 1;	

		switch(Manual_Status->Fly_or_Climb_Status)
		{
			case FLY:
			{
			//爬行机构电机控制--停止爬行
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_Pulse = 0;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC4Init(TIM3, &TIM_OCInitStruct);
			//拍打机构电机控制--使能，占空比设置
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
				Manual_Status->Fly_Pulse = Motion_Pulse;
				TIM_OCInitStruct.TIM_Pulse = Control_Pulse;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

				TIM_OC3Init(TIM3, &TIM_OCInitStruct);

			};break;
			case CLIMB:
			{
			//拍打机构电机控制--停止
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_Pulse = 0;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
				TIM_OC3Init(TIM3, &TIM_OCInitStruct);

			//爬行机构电机控制--使能，占空比设置
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
				Manual_Status->Climb_Pulse = Motion_Pulse; 
				TIM_OCInitStruct.TIM_Pulse = Control_Pulse;
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
				Manual_Status->Climb_Pulse = 0;
			//爬行机构电机控制--停止爬行
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				//TIM_OCInitStruct.TIM_Pulse = 150;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC4Init(TIM3, &TIM_OCInitStruct);
				Manual_Status->Fly_Pulse = 0;
			};break;
		}
/*//舵机位置指令处理
		switch(Actuator_Status->Control_Status)
		{
			case ROLL:
			{
			//俯仰--占空比设置
				Actuator_Status->RightServo_Pulse =  Control_Pulse;
				TIM_SetCompare2(TIM2,Actuator_Status->RightServo_Pulse);
			};break;
			case PITCH:
			{
			//翻滚--占空比设置
				Actuator_Status->LeftServo_Pulse =  Control_Pulse;
				TIM_SetCompare3(TIM2,Actuator_Status->LeftServo_Pulse );
			};break;
			case YAW:
			{
			//偏航--占空比设置
				Actuator_Status->MidServo_Pulse = Control_Pulse;
				TIM_SetCompare4(TIM2,Actuator_Status->MidServo_Pulse);
			};break;
			default:;break;
		}
*/
}

void Motor_action(const MANUAL_STATUS* Manual_status, const int16_t task_delay_num)
{
	TIM_OCInitTypeDef TIM_OCInitStruct;
	static u8 count_num = 0;
	uint8_t Fly_current_pulse = TIM3->CCR3;		//读取当前的占空比的值
	uint8_t Climb_current_pulse = TIM3->CCR4;	//读取当前的占空比的值
	if( Fly_current_pulse != Manual_status->Fly_Pulse )
	{
		if(Fly_current_pulse < Manual_status->Fly_Pulse)
		{
			if(count_num == 0)
			{
				if(task_delay_num >= 20)
					count_num = task_delay_num / 20;
				else
				{
					count_num = 20 / task_delay_num;	
					Fly_current_pulse ++;
				}
			}
			else
			{
				if(task_delay_num >= 20)
				{
					if((Manual_status->Fly_Pulse - Fly_current_pulse) > count_num)
						Fly_current_pulse += count_num;
					else 
						Fly_current_pulse = Manual_status->Fly_Pulse;
				}
				else
				{
					if((Manual_status->Fly_Pulse - Fly_current_pulse) > 0)
						count_num--;
				}
			}
		}
		else
			Fly_current_pulse = Manual_status->Fly_Pulse; 
	}
	
	if( Climb_current_pulse != Manual_status->Climb_Pulse)
	{
		if(Climb_current_pulse < Manual_status->Climb_Pulse)
		{
			if(count_num == 0)
			{
				if(task_delay_num >= 20)
					count_num = task_delay_num / 20;
				else
				{
					count_num = 20 / task_delay_num;	
					Climb_current_pulse ++;
				}
			}
			else
			{
				if(task_delay_num >= 20)
				{
					if((Manual_status->Fly_Pulse - Climb_current_pulse) > count_num)
						Climb_current_pulse += count_num;
					else 
						Climb_current_pulse = Manual_status->Climb_Pulse;
				}
				else
				{
					if((Manual_status->Climb_Pulse - Climb_current_pulse) > 0)
						count_num--;
				}
			}
		}
		else
			Fly_current_pulse = Manual_status->Climb_Pulse; 
	}

	if( Fly_current_pulse > 3 )								 		//拍打电机Enable，并设置占空比
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		if( Manual_status->Fly_Pulse <= 100 )
			TIM_OCInitStruct.TIM_Pulse = Fly_current_pulse;		
		else
			TIM_OCInitStruct.TIM_Pulse = 100;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	}
	else
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStruct.TIM_Pulse = 0;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	}
	if( Climb_current_pulse > 3 )									 //爬行电机Enable，并设置占空比
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		if(Manual_status->Climb_Pulse <= 100)
			TIM_OCInitStruct.TIM_Pulse = Climb_current_pulse;
		else
			TIM_OCInitStruct.TIM_Pulse = 100;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	}
	else
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStruct.TIM_Pulse = 0;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	}
	TIM_SetCompare2(TIM2,Manual_status->LeftServo_Pulse);						//刷新left舵机占空比
	TIM_SetCompare3(TIM2,Manual_status->MidServo_Pulse);						//刷新mid舵机占空比
	TIM_SetCompare4(TIM2,Manual_status->RightServo_Pulse);						//刷新right舵机占空比

}

/*void Motor_action(const MANUAL_STATUS* Manual_status)
{
	TIM_OCInitTypeDef TIM_OCInitStruct;

	if( Manual_status->Fly_Pulse > 3 )								 		//拍打电机Enable，并设置占空比
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		if( Manual_status->Fly_Pulse <= 100 )
			TIM_OCInitStruct.TIM_Pulse = Manual_status->Fly_Pulse;		
		else
			TIM_OCInitStruct.TIM_Pulse = 100;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	}
	else
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStruct.TIM_Pulse = 0;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	}
	if( Manual_status->Climb_Pulse > 3 )									 //爬行电机Enable，并设置占空比
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		if(Manual_status->Climb_Pulse <= 100)
			TIM_OCInitStruct.TIM_Pulse = Manual_status->Climb_Pulse;
		else
			TIM_OCInitStruct.TIM_Pulse = 100;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	}
	else
	{
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStruct.TIM_Pulse = 0;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	}
	TIM_SetCompare2(TIM2,Manual_status->LeftServo_Pulse);						//刷新left舵机占空比
	TIM_SetCompare3(TIM2,Manual_status->MidServo_Pulse);						//刷新mid舵机占空比
	TIM_SetCompare4(TIM2,Manual_status->RightServo_Pulse);						//刷新right舵机占空比
}*/

void Flight_command(const FLIGHT_STATUS* Flight_Status)
{
	
}




void Motor_Reset(MANUAL_STATUS* manual_status)
{

			
}
