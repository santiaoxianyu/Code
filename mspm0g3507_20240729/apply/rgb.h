#ifndef __RGB_H
#define __RGB_H


#define BLING_R_PORT GPIO_RGB_PORT
#define BLING_G_PORT GPIO_RGB_PORT
#define BLING_B_PORT GPIO_RGB_PORT



#define BLING_R_PIN GPIO_RGB_RED_PIN//��ɫ
#define BLING_G_PIN GPIO_RGB_GREEN_PIN//��ɫ
#define BLING_B_PIN GPIO_RGB_BLUE_PIN//��ɫ


typedef struct
{
  uint16_t contiune_t;//��˸����ʱ��
  uint16_t period;//��˸����
  float  percent;//��˸ռ�ձ�
  uint16_t  cnt;//��˸������
  GPIO_Regs *port; //�˿�
  uint32_t pin;//����
  uint8_t endless_flag;//�޾�ģʽ
}Bling_Light;


void rgb_init(void);
void bling_working(uint16_t bling_mode);

void bling_set(Bling_Light *light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint8_t Flag);
void rgb_start_bling(void);
		
extern Bling_Light light_red,light_green,light_blue;
	
#endif

