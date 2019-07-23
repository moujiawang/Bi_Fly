#include "stm32f10x.h"
#include "delay.h"
#include "nrf_protocol.h"
#include "DTU.h"
#include "motor.h"
#include "24l01.h" 	
#include <stdlib.h>
#include <string.h>
#include "upload_state_machine.h"
#include "IncPID.h"



volatile uint32_t Last_count = 0;
volatile uint32_t New_count = 0;
int32_t Receive_length[10] = {0};
volatile uint8_t Channel_Num = 0;
volatile uint8_t Update_Num = 0;
volatile uint8_t Channel_status = End;
volatile uint8_t Receive_complete_flag = 0;
volatile uint8_t Receive_Wrong_flag = 0;
//while loop time control flag，when Do_flag = 0，while loop can execute
volatile uint8_t Do_Flag = 0;											


u16 T = 0;
//u16 t=0;	
u8 Rx_buf[RX_PLOAD_WIDTH] = {0};
u8 Tx_buf[TX_PLOAD_WIDTH] = {0};
u8 RX_Result;
u8 TX_Result;
u8 Mode_ID;
<<<<<<< HEAD
TX_FLAG Tx_Flag = TX_WAIT;		
=======
TX_FLAG Tx_Flag = 0;	

//系统的状态量
SYS_STATUS SYS_Status; 	
>>>>>>> 67753ccbe8b87348627ef750ab8053a7a981ca88

#define MODE_STATUS (SYS_Status.DTU_NRF_Status|0x38)

#define MODE_STATUS (SYS_Status.DTU_NRF_Status|0x38)

int main(void)
{
	u8 sta_tmp = 0;
	u8 rx_len = 0;
	u8 Mode_ID = 0xff;

	delay_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
//	DTU_init();																		//数传模块初始化
	motor_init();																	//电机控制定时器初始化
	TIM_Cmd(TIM2, ENABLE);															//使能TIM2
	TIM_Cmd(TIM3, ENABLE);															//使能TIM3
	NRF24L01_Init(TX_MODE);    														//初始化NRF24L01为发送模式    
	load_config();  																//从flash中读取配置信息 -->eeprom.c
	Initial_UART1(115200L);															//初始化串口
	IncPID_Init(&SYS_Status.PID_Paras);												//初始化PID参数
	////////////////IMU/////////////////
	imu_fusion_init(&SYS_Status.imu_fusion_module);
	////////////////////////////////////
	while( NRF24L01_Check() == 1);
	SYS_Status.DTU_NRF_Status |= NRF_ON;											//有NRF在线,更新标志位
	
	Command_patch(Tx_buf, &SYS_Status, START_MODE);		//更新Tx_buf
	do
	{
		rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
		if((rx_len>0)&&(rx_len<33))
		{
			SYS_Status.DTU_NRF_Status |= NRF_CONNECTED;
			break;
		}
	}
	while(1);//只有当NRF24L01在线并且通讯正常时才会跳过次循环																		//直至握手成功后，退出循环
	SYS_Status.DTU_NRF_Status = SYS_Status.DTU_NRF_Status & 0xC7;//模式信息更新为 START MODE
	
	
/*	while(Receive_complete_flag == 0 )												//等待DTU第一次握手
	{
		i++;
		delay_ms(2);
		if( i == 255) 	
		{
			SYS_status.DTU_NRF_Status 
			break;
		}
	};										
	Receive_complete_flag = 0;*/		
/*	NRF24L01_PowerDown_Mode();
	NRF24L01_RX_Mode();
	while(NRF24L01_RxPacket(Rx_buf));
	Mode_ID = Command_dispatch(Rx_buf, &Actuator_Status, &Motion_Status, &PID_paras);//解包，刷新控制参数，赋值给各自的控制参数结构体中
	switch(Mode_ID)
	{
		case 0x20:Actuator_command(&Actuator_Status);break;
		case 0x2f:Motion_command(&Motion_Status);break;
		case 0x30:PID_command(&PID_paras);break;
		defualt:;
	}
*/
	while(1)
	{
		//////////////////IMU////////////////////
		imu_fusion_do_run(&SYS_Status.imu_fusion_module);
		/////////////////////////////////////////
		if(Tx_Flag == TX_GO)
		{
<<<<<<< HEAD
			Command_patch(Tx_buf, &PID_Paras, &Actuator_Status, &imu_fusion_module, MODE_STATUS);		//打包数据，更新Tx_buf，准备发送
=======
			Command_patch(Tx_buf, &SYS_Status, MODE_STATUS);		//打包数据，更新Tx_buf，准备发送
>>>>>>> 67753ccbe8b87348627ef750ab8053a7a981ca88
			rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
			if((rx_len>0)&&(rx_len<33))
			{	
				//刷新状态标志和模式信息
				SYS_Status.DTU_NRF_Status |= NRF_CONNECTED;
<<<<<<< HEAD
				Command_dispatch(Rx_buf, &Actuator_Status, &Motion_Status, &PID_Paras,&SYS_Status);
=======
				Command_dispatch(Rx_buf,&SYS_Status);
>>>>>>> 67753ccbe8b87348627ef750ab8053a7a981ca88
			}
			else
			{
				SYS_Status.DTU_NRF_Status &= (~NRF_CONNECTED);
				NRF24L01_FlushTX();
				if(NRF24L01_Check())
				{
					SYS_Status.DTU_NRF_Status |= FAULT_MODE;
				}
			}
																
		}
		switch(MODE_STATUS)
		{
			case START_MODE:
			{
				//init_motor();
			};break;
<<<<<<< HEAD
			case ACTUATOR_MODE:Actuator_command(&Actuator_Status);break;
			case MOTION_MODE  :Motion_command(&Motion_Status);break;
			case PID_MODE     :PID_command(&PID_Paras);break;
			case FAULT_MODE   :
			{
				if(NRF24L01_Check())
						SYS_Status.DTU_NRF_Status &= NRF_OFF;
				else
						SYS_Status.DTU_NRF_Status |= NRF_ON;
				
				Command_patch(Tx_buf, &PID_Paras, &Actuator_Status, &imu_fusion_module, START_MODE);		//更新Tx_buf
				do
				{
					rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
					if((rx_len>0)&&(rx_len<33))
					{
						SYS_Status.DTU_NRF_Status |= NRF_CONNECTED;
						break;
					}
				}
				while(1);//只有当NRF24L01在线并且通讯正常时才会跳过次循环
				SYS_Status.DTU_NRF_Status = (SYS_Status.DTU_NRF_Status & 0xc7)|Rx_buf[1];
			};
=======
			case ACTUATOR_MODE:
			{
				if(Tx_Flag == TX_GO)//有指令刷新时执行一次							
				{
					Actuator_command(&SYS_Status.Actuator_Status);break;
				}
			}
			case MOTION_MODE  :Motion_command(&SYS_Status.Motion_Status);break;
			case PID_MODE     :PID_command(&SYS_Status);break;
			case FAULT_MODE   :Fault_command(&SYS_Status);break;
>>>>>>> 67753ccbe8b87348627ef750ab8053a7a981ca88
		}
		Tx_Flag = TX_WAIT;//复位心跳标志位，等待下次进入	
		

//		if( NRF24L01_RxPacket(Rx_buf) == 0 )
//		{
//			Command_dispatch(Rx_buf, &Actuator_Status, &Motion_Status, &PID_paras);
//			switch(Mode_ID)
//			{
//				case 0x20:Actuator_command(&Actuator_Status);break;
//				case 0x2f:Motion_command(&Motion_Status);break;
//				case 0x30:PID_command(&PID_paras);break;
//				defualt:;
//			}
//		};
		

/*		if(Do_Flag == 0)
		{										
			//////////////////IMU////////////////////
			imu_fusion_do_run(&imu_fusion_module);
			/////////////////////////////////////////
			Command_patch(Tx_buf, &PID_paras, &Actuator_Status, &imu_fusion_module);
			NRF24L01_PowerDown_Mode();
			NRF24L01_TX_Mode();
			TX_Result = NRF24L01_TxPacket(Tx_buf);
			NRF24L01_PowerDown_Mode();
			NRF24L01_RX_Mode();
			Do_Flag = ~0;	
		}
		if(Receive_complete_flag == 1)
		{
			Command_manage(Receive_length, &Actuator_Status);							
			Receive_complete_flag = 0;
		}
		
		RX_Result = NRF24L01_RxPacket(Rx_buf);
//		Command_dispatch(Rx_buf);
		if(RX_Result == 0)
		{
			Actuator_Status.test = Rx_buf[0];
		}*/
		
	};
}

/*void TIM4_IRQHandler()
{
	if( TIM_GetITStatus(TIM4,TIM_IT_CC2) == 1 )
	{

		New_count = TIM_GetCapture2(TIM4);
		if( Channel_status != End )
		{
			Receive_length[Channel_Num] = New_count + ((uint32_t)Update_Num * TIM4_PERIOD) - Last_count;
			if(( Receive_length[Channel_Num] <= 8000 ) || ( Receive_length[Channel_Num] >= 16000 ))
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
}*/


