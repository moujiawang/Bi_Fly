#include "nrf_protocol.h"
#include "task.h"
#include "IMU.h"
#include "motor.h"
#include "24l01.h" 	



extern u8 Rx_buf[RX_PLOAD_WIDTH];
extern u8 Tx_buf[TX_PLOAD_WIDTH];





void Start_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;

	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//�����˽��ٶȣ��Ǽ��ٶ�
	///////////////////////////////////////// 
	Command_patch(Tx_buf, SYS_status,START_MODE);	 		//������ݣ�����Tx_buf��׼������
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//����Ӧ���źųɹ�
	{	
		//ˢ��״̬��־��ģʽ��Ϣ
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//������ַ�����
	}
	else													//����Ӧ���ź�ʧ��
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRFģ��������оƬû������			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRFģ��������оƬ��������	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	


}

void Manual_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//�����˽��ٶȣ��Ǽ��ٶ�
	/////////////////////////////////////////
	Command_patch(Tx_buf, SYS_status, MANUAL_MODE);	 		//������ݣ�����Tx_buf��׼������
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//����Ӧ���źųɹ�
	{	
		//ˢ��״̬��־��ģʽ��Ϣ
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//������ַ�����
		Manual_command(&SYS_status->Manual_Status);			//���ݽ�����ָ�ִ��ָ���
	}
	else													//����Ӧ���ź�ʧ��
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRFģ��������оƬû������			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRFģ��������оƬ��������	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	

}

void Flight_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//�����˽��ٶȣ��Ǽ��ٶ�
	/////////////////////////////////////////
	Command_patch(Tx_buf, SYS_status, FLIGHT_MODE);	 		//������ݣ�����Tx_buf��׼������
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//����Ӧ���źųɹ�
	{	
		//ˢ��״̬��־��ģʽ��Ϣ
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//������ַ�����
		Flight_command(&SYS_status->Flight_Status);			//���ݽ�����ָ�ִ��ָ���
	}
	else													//����Ӧ���ź�ʧ��
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRFģ��������оƬû������			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRFģ��������оƬ��������	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	

}

void Tuning_task(SYS_STATUS *SYS_status)
{
	u8 rx_len = 0;
	//////////////////IMU////////////////////
	imu_fusion_do_run(&SYS_status->imu_fusion_module);		//�����˽��ٶȣ��Ǽ��ٶ�
	/////////////////////////////////////////
	Command_patch(Tx_buf, SYS_status, TUNING_MODE);	 		//������ݣ�����Tx_buf��׼������
	rx_len = NRF24L01_Tx_ACKwithpayload(Tx_buf, Rx_buf);
	if(rx_len != 0x00)										//����Ӧ���źųɹ�
	{	
		//ˢ��״̬��־��ģʽ��Ϣ
		SYS_status->DTU_NRF_Status |= NRF_CONNECTED;
		Command_dispatch(Rx_buf,SYS_status);				//������ַ�����
		PID_command( SYS_status );			//���ݽ�����ָ�ִ��ָ���
	}
	else													//����Ӧ���ź�ʧ��
	{
		SYS_status->DTU_NRF_Status &= (~NRF_CONNECTED);
		NRF24L01_FlushTX();
		if(NRF24L01_Check())
		{
			SYS_status->DTU_NRF_Status &= NRF_OFF;			//NRFģ��������оƬû������			
		}
		else
		{
			SYS_status->DTU_NRF_Status |= NRF_ON;			//NRFģ��������оƬ��������	
		}
		SYS_status->DTU_NRF_Status = (SYS_status->DTU_NRF_Status & 0xc7) | FAULT_MODE;
	}	
}

void Fault_task(SYS_STATUS *SYS_status)
{
	
}

void UpdateIMU_task(SYS_STATUS *SYS_status)
{
							
}