#include "stm32f10x.h"
#include "delay.h"
#include "DTU.h"
#include "motor.h"
#include "24l01.h" 	
#include <stdlib.h>
#include <string.h>
#include "upload_state_machine.h"
#include "nrf_protocol.h"


volatile uint32_t Last_count = 0;
volatile uint32_t New_count = 0;
int32_t Receive_length[10] = {0};
volatile uint8_t Channel_Num = 0;
volatile uint8_t Update_Num = 0;
volatile uint8_t Channel_status = End;
volatile uint8_t Receive_complete_flag = 0;
volatile uint8_t Receive_Wrong_flag = 0;
volatile uint8_t Do_Flag = 0;											//while loop time control flag，when Do_flag = 0，while loop can execute

u16 T = 0;
u16 t=0;	
u8 Rx_buf[RX_PLOAD_WIDTH];
u8 Tx_buf[TX_PLOAD_WIDTH];
u8 RX_Result;
u8 TX_Result;

IMUFusion imu_fusion_module;
MOTION_STATUS Motion_Status;

int main(void)
{
	delay_init();
	Receive_length[0] = 1-2;
	Receive_length[0] = abs(Receive_length[0]);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	DTU_init();															//数传模块初始化
	motor_init();														//电机控制定时器初始化
	TIM_Cmd(TIM2, ENABLE);												//使能TIM2
	TIM_Cmd(TIM3, ENABLE);												//使能TIM3
	
	while(Receive_complete_flag == 0);									//等待第一次指令接收完成
	Command_manage(Receive_length,&Motion_Status);
	Receive_complete_flag = 0;
	

	NRF24L01_Init();    												//初始化NRF24L01 
	
	////////////////IMU/////////////////
	imu_fusion_init(&imu_fusion_module);
	////////////////////////////////////
	while(1)
	{
		if(Do_Flag == 0)
		{
			if(Receive_complete_flag == 1)
			{
				Command_manage(Receive_length, &Motion_Status);							
				Receive_complete_flag = 0;
			}
			Do_Flag = ~0;											
			//////////////////IMU////////////////////
			imu_fusion_do_run(&imu_fusion_module);
//			IMU_getYawPitchRoll(&imu_fusion_module.estimator, imu_fusion_module.ypr); //姿态更新
//			imu_fusion_module.math_hz++; //解算次数 ++
//			//	base_timer_get_us();
//			//	SysTick->VAL;
//			//	micros();
//			if((micros()-imu_fusion_module.system_micrsecond)>upload_time)
//			{
//				update_upload_state(&imu_fusion_module.upload_state, imu_fusion_module.ypr, &imu_fusion_module.math_hz);
//				imu_fusion_module.system_micrsecond = micros();	 //取系统时间 单位 us 
//			}
			/////////////////////////////////////////
			
			
//			NRF24L01_PowerDown_Mode();
//			NRF24L01_TX_Mode();
//			TX_Result = NRF24L01_TxPacket(Tx_buf);
//			NRF24L01_PowerDown_Mode();
//			NRF24L01_RX_Mode();
//			RX_Result = NRF24L01_RxPacket(Rx_buf);
//			command_dispatch(Rx_buf);
			

		}
	};
}

void TIM4_IRQHandler()
{
	if( TIM_GetITStatus(TIM4,TIM_IT_CC2) == 1 )
	{

		New_count = TIM_GetCapture2(TIM4);
		if( Channel_status != End )
		{
			Receive_length[Channel_Num] = New_count + ((uint32_t)Update_Num * TIM4_PERIOD) - Last_count;
			if((Receive_length[Channel_Num] <= 8000) || (Receive_length[Channel_Num] >= 16000))
				Receive_Wrong_flag = 1;
			Channel_Num++;
		}
		else
		{	
			Channel_Num = 0;
			Channel_status = 0;										//Command_status ！= END 
		}
		Last_count = New_count;
		Update_Num = 0;
	}
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) == 1)
	{
		T++;	
		if(T == 50)													//50 * 1ms 执行一次while循环
		{
			Do_Flag = ~Do_Flag;										//Do_Flag刷新取反
			T = 0;
		}					
		Update_Num++;
		if( Update_Num > 3 )										//接收到的通道指令最大值为大于16000，小于17000，所以TIM4在通道指令没发完前最多可能溢出3次
		{
			if((Channel_Num == 10) && (Receive_Wrong_flag == 0))	//接受完9条通道的指令，且接收到的指令中没有异常的指令（小于8000（最小指令值）和大于16000（最大指令值））
			{
				Receive_complete_flag = 1;							//指令接受完成标志位置位
			}
			else													//如果连续两次计数器溢出还没有一次的捕获，说明在两条指令之间，但是如果指令数不等于10说明前面的数据有通道的指令被丢弃或者误识通道指令，所以后面程序将收到的通道指令全部丢弃；
			{
				memset(&Receive_length, 0, sizeof(Receive_length));	//将Receive_Length数组清零
				Receive_Wrong_flag = 0;
			}
			Channel_status = End;									//指令接收完成标志位没有置位，不会更新PWM占空比
		}
		TIM_ClearFlag(TIM4,TIM_IT_Update);
	}
}


