#include "ti_msp_dl_config.h"
#include "ntimer.h"
#include "npwm.h"





/***************************************
������:	void PWM_Output((uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4))
˵��: uint16_t width1-�������������1
			uint16_t width2-�������������2
			uint16_t width3-�������������3
			uint16_t width4-�������������4
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
	uint16_t pwm[4]={0};
	pwm[0]=width1;
	pwm[1]=width2;
	pwm[2]=width3;
	pwm[3]=width4;
	
	DL_TimerA_setCaptureCompareValue(PWM_0_INST, pwm[0], GPIO_PWM_0_C0_IDX);//TIMA0-CH0-PB14 0~1000
	DL_TimerA_setCaptureCompareValue(PWM_0_INST, pwm[1], GPIO_PWM_0_C1_IDX);//TIMA0-CH1-PA3
	DL_TimerA_setCaptureCompareValue(PWM_0_INST, pwm[2], GPIO_PWM_0_C2_IDX);//TIMA0-CH2-PA7  0~999
	DL_TimerA_setCaptureCompareValue(PWM_0_INST, pwm[3], GPIO_PWM_0_C3_IDX);//TIMA0-CH3-PA4
}      

/***************************************
������:	void steer_servo_pwm_m1p0(uint16_t us)
˵��: uint16_t us-�������ֵ
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void steer_servo_pwm_m1p0(uint16_t us)
{
	Reserved_PWM5_Output(us);
}

/***************************************
������:	void steer_servo_pwm_m1p1(uint16_t us)
˵��: uint16_t us-�������ֵ
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void steer_servo_pwm_m1p1(uint16_t us)
{
	Reserved_PWM6_Output(us);
}

/***************************************
������:	void steer_servo_pwm_m1p2(uint16_t us)
˵��: uint16_t us-�������ֵ
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void steer_servo_pwm_m1p2(uint16_t us)
{
	Reserved_PWM7_Output(us);
}

/***************************************
������:	void steer_servo_pwm_m1p3(uint16_t us)
˵��: uint16_t us-�������ֵ
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void steer_servo_pwm_m1p3(uint16_t us)
{
	Reserved_PWM8_Output(us);
}
