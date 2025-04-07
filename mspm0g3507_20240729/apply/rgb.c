#include "ti_msp_dl_config.h"
#include "headfile.h"
#include "rgb.h"




	
	
Bling_Light light_red={
	.port=BLING_R_PORT,
	.pin=GPIO_RGB_RED_PIN,
};

Bling_Light light_green={
	.port=BLING_G_PORT,
	.pin=GPIO_RGB_GREEN_PIN,
};

Bling_Light light_blue={
	.port=BLING_B_PORT,
	.pin=GPIO_RGB_BLUE_PIN,
};


/***************************************
������:	void rgb_init(void)
˵��: rgb�����ڵ�IO��ʼ��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void rgb_init(void)
{	
	rgb_start_bling();            //����LEDԤ��ʾ
}


/***************************************************
������: void Bling_Set(Bling_Light *Light,
uint32_t Continue_time,//����ʱ��
uint16_t Period,//����100ms~1000ms
float Percent,//0~100%
uint16_t  Cnt,
GPIO_TypeDef* Port,
uint16_t Pin
,uint8_t Flag)
˵��:	״ָ̬ʾ�����ú���
���:	ʱ�䡢���ڡ�ռ�ձȡ��˿ڵ�
����:	��
��ע:	�����ʼ����ʼ������
����:	��������
****************************************************/
void bling_set(Bling_Light *light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint8_t Flag)
{
  light->contiune_t=(uint16_t)(Continue_time/5);//����ʱ��
  light->period=Period;//����
  light->percent=Percent;//ռ�ձ�
  light->cnt=Cnt;
  //light->port=Port;//�˿�
  //light->pin=Pin;//����
  light->endless_flag=Flag;//�޾�ģʽ
}

/***************************************************
������: void Bling_Process(Bling_Light *Light)//��˸�����߳�
˵��:	״ָ̬ʾ��ʵ��
���:	״̬�ƽṹ��
����:	��
��ע:	�����ʼ����ʼ������
����:	��������
****************************************************/
void bling_process(Bling_Light *light)//��˸�����߳�
{
  if(light->contiune_t>=1)  light->contiune_t--;
  else DL_GPIO_clearPins(light->port, light->pin);
  if(light->contiune_t!=0//��ʱ��δ��0
     ||light->endless_flag==1)//�ж��޾�ģʽ�Ƿ���
  {
    light->cnt++;
    if(5*light->cnt>=light->period) light->cnt=0;//��������
    if(5*light->cnt<=light->period*light->percent)
      DL_GPIO_setPins(light->port, light->pin);//�øߣ���
    else  DL_GPIO_clearPins(light->port, light->pin);//�õͣ���
  }
}



void nGPIO_SetBits(Bling_Light *light)
{
	DL_GPIO_setPins(light->port,light->pin);
}

void nGPIO_ResetBits(Bling_Light *light)
{
	DL_GPIO_clearPins(light->port,light->pin);
}


/***************************************************
������: Bling_Working(uint16 bling_mode)
˵��:	״ָ̬ʾ��״̬��
���:	��ǰģʽ
����:	��
��ע:	�����ʼ����ʼ������
����:	��������
****************************************************/
void bling_working(uint16_t bling_mode)
{
  if(bling_mode==0)//ȫ��
  {
    bling_process(&light_blue);
    bling_process(&light_red);
    bling_process(&light_green);
  }
  else if(bling_mode==3)//ȫ��
  {
    DL_GPIO_clearPins(BLING_R_PORT,   GPIO_RGB_RED_PIN);
    DL_GPIO_clearPins(BLING_G_PORT,   GPIO_RGB_GREEN_PIN);
    DL_GPIO_clearPins(BLING_B_PORT,   GPIO_RGB_BLUE_PIN);
  } 
}

/***************************************
������:	void rgb_start_bling(void)
˵��: �������ʼ��˸
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void rgb_start_bling(void)
{
	bling_set(&light_red  ,1000,500,0.5,0,0);//��ɫ
  bling_set(&light_green,1000,500,0.5,0,0);//��ɫ
  bling_set(&light_blue ,1000,500,0.5,0,0);//��ɫ
}
