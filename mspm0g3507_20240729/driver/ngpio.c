#include "ti_msp_dl_config.h"
#include "ngpio.h"


_laser_light beep;

/***************************************
������:	void GPIO_Init(void)
˵��: GPIO��ʼ��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void nGPIO_Init(void)
{
  beep.port = PORTA_PORT;
  beep.pin = PORTA_BEEP_PIN;
  beep.period = 200; //200*5ms
  beep.light_on_percent = 0.5f;
	
  beep.reset = 1;
  beep.times = 0;
}

/***************************************
������:	laser_light_work(_laser_light *light)
˵��: gpio����״̬��
���:	_laser_light *light-gpio���ƽṹ��
����:	��
��ע:	��
����:	��������
***************************************/
void laser_light_work(_laser_light *light)
{
	if(light->reset==1)
	{
		light->reset=0;
		light->cnt=0;
		light->times_cnt=0;//��������������
		light->end=0;
	}
	
	if(light->times_cnt==light->times)
	{
		light->end=1;
		return;
	}

	light->cnt++;
	if(light->cnt<=light->period*light->light_on_percent)
	{
		DL_GPIO_setPins(light->port, light->pin);
	}
	else if(light->cnt<light->period)
	{
		DL_GPIO_clearPins(light->port, light->pin);
	}
	else//��ɵ���һ��
	{
		light->cnt=0;
		light->times_cnt++;
	}
}
//	uint16_t times;				 //Ԥ����˸�ܴ���
//	uint8_t  reset;				 //��˸���̸�λ��־
//	uint16_t cnt;					 //��˸���Ƽ�����
//	uint16_t times_cnt;		 //��¼����˸����
//	uint8_t  end;					 //��˸��ɱ�־λ
//	GPIO_Regs *port;		   //��˸���ڵĶ˿�
//	uint32_t pin;					 //��˸���ڵ�GPIO
//	uint32_t period;			 //��˸����
//	float light_on_percent;//���������ڵ���ʱ��ٷֱ�


void buzzer_setup(uint32_t _period, float _light_on_percent, uint16_t _times)
{
  beep.period = _period / 5; //20*5ms
  beep.light_on_percent = _light_on_percent;
  beep.reset = 1;
  beep.times = _times;
}