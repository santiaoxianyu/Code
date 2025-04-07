#include "ti_msp_dl_config.h"
#include "datatype.h"
#include "system.h"
#include "us100.h"
#include "nqei.h"
#include "nppm.h"

mppm ppm_rc;//ppm���ݽṹ��

void nPPM_Handler(void);
/***************************************
������:	void PPM_Init(void)
˵��: PPMӲ����Դ��ʼ��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void PPM_Init(void)
{	
	//NVIC_ClearPendingIRQ(PORTA_INT_IRQN);
	NVIC_ClearPendingIRQ(PORTB_INT_IRQN);
	//NVIC_EnableIRQ(PORTA_INT_IRQN);//GPIOA�ж�ʹ��
	NVIC_EnableIRQ(PORTB_INT_IRQN);//GPIOB�ж�ʹ��
}


/***************************************
������:	void nPPM_Handler(void)
˵��: PPM�жϷ�����
���:	��
����:	��
��ע:	��
����:	��������  mk0
***************************************/
uint16 PPM_buf[8]={0}; 
uint32_t last_ppm_time=0,now_ppm_time=0;
uint8_t ppm_ready=0,ppm_sample_cnt=0,ppm_prase=0;
uint16_t ppm_time_delta=0;	
void nPPM_Handler(void)
{
  last_ppm_time=now_ppm_time;//ϵͳ����ʱ���ȡ����λus
  now_ppm_time=micros();//��λus	
  ppm_time_delta=now_ppm_time-last_ppm_time;//��ȡʱ����
  //PPM������ʼ
	if(ppm_ready==1)
	{
		if(ppm_time_delta>=2400)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������
			//���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
		{
			ppm_prase = 1;
			ppm_sample_cnt=0;		
		}
		else if(ppm_time_delta>=900&&ppm_time_delta<=2100&&ppm_prase==1)
		{         
			PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//��Ӧͨ��д�뻺����       
			if(ppm_sample_cnt>=8)//���ν�������
			{
				memcpy(ppm_rc.buf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_prase=0;
				ppm_rc.update_flag=1;
			}
		}
	}
	else if(ppm_time_delta>=2400)//֡������ƽ����2ms=2000us
	{
		ppm_ready=1;
		ppm_prase=1;
		ppm_sample_cnt=0;
	}
}



void GROUP1_IRQHandler(void)
{
//	if(DL_Interrupt_getStatusGroup(DL_INTERRUPT_GROUP_1,DL_INTERRUPT_GROUP1_GPIOB))
//	{
//		if(DL_GPIO_getEnabledInterruptStatus(PORTB_PORT, PORTB_RIGHT_PULSE_PIN))
//		{
//			QEI0_IRQHandler();
//			DL_GPIO_clearInterruptStatus(PORTB_PORT, PORTB_RIGHT_PULSE_PIN);
//		}		
//		
//		if(DL_GPIO_getEnabledInterruptStatus(PORTB_PORT, PORTB_LEFT_PULSE_PIN))
//		{
//			QEI1_IRQHandler();
//			DL_GPIO_clearInterruptStatus(PORTB_PORT, PORTB_LEFT_PULSE_PIN);
//		}
//		DL_Interrupt_clearGroup(DL_INTERRUPT_GROUP_1, DL_INTERRUPT_GROUP1_GPIOB);		
//	}
//	
////	if(DL_Interrupt_getStatusGroup(DL_INTERRUPT_GROUP_1, DL_INTERRUPT_GROUP1_GPIOA))
////	{
////		if(DL_GPIO_getEnabledInterruptStatus(PORTA_PORT, PORTA_PPM_PIN))
////		{
////			//nPPM_Handler();
////			DL_GPIO_clearInterruptStatus(PORTA_PORT, PORTA_PPM_PIN);
////		}	
////		DL_Interrupt_clearGroup(DL_INTERRUPT_GROUP_1, DL_INTERRUPT_GROUP1_GPIOA);		
////	}
QEI0_IRQHandler();
}


