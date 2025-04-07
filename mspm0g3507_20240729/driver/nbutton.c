#include "ti_msp_dl_config.h"
#include "system.h"
#include "ngpio.h"
#include "nbutton.h"


#define keydown_port  KEYB_PORT
#define keydown_pin   KEYB_S3_PIN

#define keyup_port    KEYA_PORT
#define keyup_pin     KEYA_S2_PIN


rc_buttton _button;

/***************************************
������:	void Button_Init(void)
˵��: ���ذ�����ʼ��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void Button_Init(void)
{
  _button.state[UP].port = keyup_port;
  _button.state[UP].pin = keyup_pin;
  _button.state[UP].value = 1;
  _button.state[UP].last_value = 1;

  _button.state[DOWN].port = keydown_port;
  _button.state[DOWN].pin = keydown_pin;
  _button.state[DOWN].value = 1;
  _button.state[DOWN].last_value = 1;

  //3D
  _button.state[UP_3D].port = PORTB_PORT;
  _button.state[UP_3D].pin = PORTB_D3_5_PIN;
  _button.state[UP_3D].value = 1;
  _button.state[UP_3D].last_value = 1;

  _button.state[DN_3D].port = PORTB_PORT;
  _button.state[DN_3D].pin = PORTB_D3_1_PIN;
  _button.state[DN_3D].value = 1;
  _button.state[DN_3D].last_value = 1;

  _button.state[LT_3D].port = PORTB_PORT;
  _button.state[LT_3D].pin = PORTB_D3_2_PIN;
  _button.state[LT_3D].value = 1;
  _button.state[LT_3D].last_value = 1;

  _button.state[RT_3D].port = PORTB_PORT;
  _button.state[RT_3D].pin = PORTB_D3_3_PIN;
  _button.state[RT_3D].value = 1;
  _button.state[RT_3D].last_value = 1;

  _button.state[ME_3D].port = PORTB_PORT;
  _button.state[ME_3D].pin = PORTB_D3_4_PIN;
  _button.state[ME_3D].value = 1;
  _button.state[ME_3D].last_value = 1;
}

/***************************************
������:	void Read_Button_State_One(button_state *button)
˵��: ������ȡ״̬��
���:	button_state *button-��ⰴ���ṹ��ָ��
����:	��
��ע:	��
����:	��������
***************************************/
void Read_Button_State_One(button_state *button)
{
  if(button->pin == KEYA_S2_PIN)  button->value = (DL_GPIO_readPins(button->port, button->pin) != 0 ? 0x00 : 0x01);
  else button->value = (DL_GPIO_readPins(button->port, button->pin) != 0 ? 0x01 : 0x00);

	if(button->value==0)
	{
		if(button->last_value!=0)//�״ΰ���
		{
			button->press_time=millis();//��¼���µ�ʱ���
			button->in_time=millis();//��¼���µ�ʱ���
			button->in_press_cnt=0;
		}
		else
		{
			if(millis()-button->press_time>KEEP_LONG_PRESS_LIMIT)//�ﵽ��������ʱ�����ƣ�������ʾ�����ɿ���
			{
//				beep.period=20;//20*5ms
//				beep.light_on_percent=0.5f;			
//				beep.reset=1;
//				beep.times=1;			
			}
			else if(millis()-button->in_time>IN_PRESS_LIMIT)//��������
			{
				button->in_time=millis();
				button->press=IN_PRESS;
				if(button->press==IN_PRESS)  button->in_press_cnt++;
			}
			
		}
	}
  else
	{
		if(button->last_value==0)//���º��ͷ�
		{
			button->release_time=millis();//��¼�ͷ�ʱ��ʱ��
			if(button->release_time-button->press_time>KEEP_LONG_PRESS_LIMIT)//������������5S
			{
				button->press=KEEP_LONG_PRESS;
				button->state_lock_time=0;
				
        buzzer_setup(1000,0.5f,4);				
			}
			else if(button->release_time-button->press_time>LONG_PRESS_LIMIT)//��������1S
			{
			  button->press=LONG_PRESS;
				button->state_lock_time=0;//5ms*300=1.5S
				
				buzzer_setup(1000,0.5f,1);
			}
			else
			{
			  button->press=SHORT_PRESS;
				button->state_lock_time=0;//5ms*300=1.5S
				
				buzzer_setup(100,0.5f,1);
			}
		}
	}
	button->last_value=button->value;
	
	if(button->press==LONG_PRESS
	 ||button->press==SHORT_PRESS)//�����ͷź󣬳����̨1.5S������Ӧ����λ����״̬
	{
	  button->state_lock_time++;
		if(button->state_lock_time>=300)
		{			
			 button->press=NO_PRESS;
			 button->state_lock_time=0;
		}
	}
}

/***************************************
������:	void read_button_state_all(void)
˵��: ��ȡ���а���״̬
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void read_button_state_all(void)
{
  for(uint16_t i=0;i<BUTTON_NUM;i++)
	{
	  Read_Button_State_One(&_button.state[i]);
	}
}

