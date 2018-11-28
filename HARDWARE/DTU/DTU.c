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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		//��ͨ�ö�ʱ��4��ʱ�� 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);			//��PB���ŵ�ʱ��
	
	GPIO_def.GPIO_Pin = GPIO_Pin_6;
	GPIO_def.GPIO_Mode = GPIO_Mode_IPD;						// 
//	GPIO_def.GPIO_OType = GPIO_OType_PP;						//�������
	GPIO_def.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_def);
	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);

//����ģ����ջ�����ҡ������100����Ӧ������0.586ms���Ƶ���������-100����Ӧ������1.414ms
	TIM_TimeBaseInitStruct.TIM_Prescaler = (9-1);				//72MHZ/9���ڶ�ʱ��������1���Ӽ����Ĵ�����Ҳ����8MHZ����ÿ����һ��ʱ��Ϊ0.000125ms
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM4_PERIOD;			//����10000��,��Ӧʱ����3.75ms
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;				//����Ϊͨ��1
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;	//����Ϊ�½��ؼ��
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;//TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;			//Capture performed each time an edge is detected on the capture input
	TIM_ICInitStruct.TIM_ICFilter = 3<<4;						//Fsampling = Fck_int, N=8;
	
	TIM_ICInit(TIM4, &TIM_ICInitStruct);		

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//�жϷ���		
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;				//���ó�ʼ������TIM4���ж�	
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;		//������ռ���ȼ�Ϊ2
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;				//������Ӧ���ȼ��������ȼ���Ϊ0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				//ʹ�ܶ�ʱ��4����ж�
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);						//ʹ��TIM4��IC1ͨ���ж�
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4, ENABLE);

}

void Command_manage(int32_t Command_length[])
{
	TIM_OCInitTypeDef TIM_OCInitStruct;
	uint8_t Control_Flag = 0;
	uint8_t FlyorClimb_Flag = 0;
	int32_t PWM_length = 0;

//�жϵ�ǰ�ĸ�����ܿ�
	if((Command_length[4]-4000) <= 7000)
		Control_Flag = PITCH;
	else 
		if( (Command_length[4]-4000) < 9000)
			Control_Flag = ROLL;
		else 
			Control_Flag = YAW;

//�жϵ�ǰ����ָ��
	if((7800 < (Command_length[6]-4000)) && ((Command_length[6]-4000) < 8200))
		FlyorClimb_Flag = STOP;
	else 
		if((Command_length[6]-4000) < 7800)
			FlyorClimb_Flag = FLY;
		else 
			FlyorClimb_Flag = CLIMB;
//�Էɻ�����ָ����д���
/*	if(FlyorClimb_Flag == FlyorClimb_lastFlag)					//����״̬û�б仯��ֻ��Ҫˢ��PWM����ռ�ձȾ���
	{
		switch(FlyorClimb_Flag)
		{
			case FLY:
			{
				PWM_length = ((abs(Command_length[1] - 12000)) >> 7)*80;
				TIM_SetCompare1(TIM3,PWM_length);
			};break;
			case CLIMB:
			{
				PWM_length = ((abs(Command_length[1] - 12000)) >> 7)*80;
				TIM_SetCompare2(TIM3,PWM_length);
			};break;
			default:;
		}
	}
	else														//����״̬�仯����Ҫ��1��ֹͣ��ǰPWM����ڵ����;(2)������������˿�;(3)ˢ��PWM����ռ�ձ�;
	{*/
		switch(FlyorClimb_Flag)
		{
			case FLY:
			{
			//���л����������--ֹͣ����
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC2Init(TIM3, &TIM_OCInitStruct);
			//�Ĵ�����������--ʹ�ܣ�ռ�ձ�����
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
				TIM_OCInitStruct.TIM_Pulse = ((abs(Command_length[1] - 12000)) >> 7)*80;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

				TIM_OC1Init(TIM3, &TIM_OCInitStruct);
//						GPIO_SetBits(GPIOA,GPIO_Pin_6);
			};break;
			case CLIMB:
			{
			//���л����������--ֹͣ����
				TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
				TIM_OC1Init(TIM3, &TIM_OCInitStruct);
//						GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			//�Ĵ�����������--ʹ�ܣ�ռ�ձ�����
				TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Enable;
				TIM_OCInitStruct.TIM_Pulse = ((abs(Command_length[1] - 12000)) >> 7)*80;
				TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

				TIM_OC2Init(TIM3, &TIM_OCInitStruct);
			};break;
			default:
			{
			//���л����������--ֹͣ����
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC1Init(TIM3, &TIM_OCInitStruct);
//						GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			//���л����������--ֹͣ����
				TIM_OCInitStruct.TIM_OutputState =  TIM_OutputState_Disable;
				TIM_OC2Init(TIM3, &TIM_OCInitStruct);
			};break;
		}
//		FlyorClimb_lastFlag = FlyorClimb_Flag;					//��¼��ǰ����״̬
//	}
	//�Զ������״̬��ָ����д���
/*	if(Control_Flag == Control_lastFlag)						//�������״̬û�б仯��ֻ��Ҫˢ��PWM����ռ�ձȾ���
	{
		switch(Control_Flag)
		{
			case ROLL:
			{
			//����--ռ�ձ�����
//				PWM_length = (Command_length[2] / 160) << 1;
				PWM_length = ((Command_length[2] -12000)/75 + 75) << 1;
				TIM_SetCompare2(TIM2,PWM_length);				
			};break;
			case PITCH:
			{
			//����--ռ�ձ�����
//				PWM_length = (Command_length[2] / 160) << 1;
				PWM_length = ((Command_length[2] -12000)/75 + 75) << 1;
				TIM_SetCompare3(TIM2,PWM_length);	
			};break;
			case YAW:
			{
			//ƫ��--ռ�ձ�����
				PWM_length = (((Command_length[2] -12000) >> 8) +75)<< 1;
				TIM_SetCompare4(TIM2,PWM_length);	
			};break;
			default:;break;
		}
	}
	else														//�������״̬�ı䣬��Ҫ��1��δѡ��Ķ���������λ��;(2)ˢ��PWM����ռ�ձ�;
	{*/
		switch(Control_Flag)
		{
			case ROLL:
			{
			//����--ռ�ձ�����
//						PWM_length = (Command_length[2] / 160) << 1;
				PWM_length = ((Command_length[2] -12000)/75 + 75) << 1;
				TIM_SetCompare2(TIM2,PWM_length);
//				PWM_length = 150;
//				TIM_SetCompare3(TIM2,PWM_length);
//				PWM_length = 150;
//				TIM_SetCompare4(TIM2,PWM_length);							
			};break;
			case PITCH:
			{
			//����--ռ�ձ�����
//				PWM_length = 150;
//				TIM_SetCompare2(TIM2,PWM_length);
//				PWM_length = (Command_length[2] / 160) << 1;
				PWM_length = ((Command_length[2] -12000)/75 + 75) << 1;
				TIM_SetCompare3(TIM2,PWM_length);
//				PWM_length = 150;
//				TIM_SetCompare4(TIM2,PWM_length);
			};break;
			case YAW:
			{
			//ƫ��--ռ�ձ�����
//				PWM_length = 150;
//				TIM_SetCompare2(TIM2,PWM_length);
//				PWM_length = 150;
//				TIM_SetCompare3(TIM2,PWM_length);
				PWM_length = (((Command_length[2] -12000) >> 8) +75)<< 1;					
				TIM_SetCompare4(TIM2,PWM_length);
			};break;
			default:;break;
		}
//		Control_lastFlag = Control_Flag;						//��¼��ǰ����״̬
//	}
}
