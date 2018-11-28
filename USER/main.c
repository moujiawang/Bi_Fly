#include "stm32f10x.h"
#include "delay.h"
#include "DTU.h"
#include "motor.h"
#include "stdlib.h"
#include "string.h"




volatile uint32_t Last_count = 0;
volatile uint32_t New_count = 0;
int32_t Receive_length[10] = {0};
volatile uint8_t Channel_Num = 0;
volatile uint8_t Update_Num = 0;
volatile uint8_t Channel_status = End;
volatile uint8_t Receive_complete_flag = 0;
//volatile uint8_t FlyorClimb_Flag = 0;		
//volatile uint8_t FlyorClimb_lastFlag;

volatile uint8_t Control_Flag = 0;		
volatile uint8_t Control_lastFlag;

int main(void)
{
	delay_init();
	DTU_init();															//����ģ���ʼ��
	motor_init();														//������ƶ�ʱ����ʼ��
//	IMU_init();	
	while(Receive_complete_flag == 0);									//�ȴ���һ��ָ��������
	Command_manage(Receive_length);
	Receive_complete_flag = 0;
	
	TIM_Cmd(TIM2, ENABLE);												//ʹ��TIM2
	TIM_Cmd(TIM3, ENABLE);												//ʹ��TIM3
	while(1)
	{
		if(Receive_complete_flag == 1)
		{
			Command_manage(Receive_length);
			Receive_complete_flag = 0;
		}
//		IMU();		
	};
}

void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4,TIM_IT_CC1) == 1)
	{
		New_count = TIM_GetCapture1(TIM4);
		if( Channel_status != End )
		{
			Receive_length[Channel_Num] = New_count + ((uint32_t)Update_Num * 30000) - Last_count;
			Channel_Num++;
		}
		else
		{	
			Channel_Num = 0;
			Channel_status = 0;										//Command_status ��= END 
		}
		Last_count = New_count;
		Update_Num = 0;
	}
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) == 1)
	{
		Update_Num++;
		if(Update_Num>1)
		{
			if(Channel_Num == 10)									//������9��ͨ����ָ��
			{
				Receive_complete_flag = 1;							//ָ�������ɱ�־λ��λ
			}
			else													//����������μ����������û��һ�εĲ���˵��������ָ��֮�䣬�������ָ����������10˵��ǰ���������ͨ����ָ�����������ʶͨ��ָ����Ժ�������յ���ͨ��ָ��ȫ��������
			{
				memset(&Receive_length, 0, sizeof(Receive_length));	//��Receive_Length��������
			}
			Channel_status = End;									//ָ�������ɱ�־λû����λ���������PWMռ�ձ�
		}
		TIM_ClearFlag(TIM4,TIM_IT_Update);
	}
}


