#include "headfile.h"
#include "user.h"
#include "us100.h"



uint8_t  com3_rx_buf[4]={0};
uint16_t com3_rx_cnt=0;
_rangefinder rangefinder;
/***************************************
������:	void us100_start(void)
˵��: us100��ഥ��ָ��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void us100_start(void)
{
UART_SendByte(UART_3_INST,0x55);
}

/***************************************
������:	float us100_get_distance(uint8 MSB,uint8 LSB)
˵��: �������ش��������ֽڻ����ʵ�ʾ���
���:	uint8 MSB-���ֽ�
		uint8 LSB-���ֽ�
����:	float ���ؾ���ֵ
��ע:	��
����:	��������
***************************************/
float us100_get_distance(uint8 MSB,uint8 LSB)
{
return (256*(MSB)+(LSB))/10.0f;//��λcm
}

/***************************************
������:	float us100_get_temperature(uint8_t data)
˵��: ��ȡ�¶�����
���:	uint8 data-�¶��ֽ�
����:	float �����¶�ֵ
��ע:	��
����:	��������
***************************************/
float us100_get_temperature(uint8_t data)
{
return (data-45)/1.0;//��
}

/***************************************
������:	void us100_statemachine(void)
˵��: us100����״̬��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void us100_statemachine(void)
{
static uint16_t us100_cnt=0;
us100_cnt++;
if(us100_cnt>=20)
{		
	us100_cnt=0;
	
	rangefinder.pre_last_distance=rangefinder.last_distance;//���ϴθ߶�
	rangefinder.last_distance=rangefinder.distance;//�ϴθ߶�
	rangefinder.distance=us100_get_distance(com3_rx_buf[0],com3_rx_buf[1]);

	rangefinder.last_vel=rangefinder.vel;
	rangefinder.vel=(rangefinder.distance-rangefinder.last_distance)/0.1f;
	rangefinder.acc=(rangefinder.vel-rangefinder.last_vel)/0.1f;
	
	com3_rx_cnt=0;
	us100_start();
}
}





/***************************************
������:	void rangefinder_init(void)
˵��: ��ഫ������ʼ��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void rangefinder_init(void)
{
float tmp_sensor_type=0;
ReadFlashParameterOne(RANGEFINDER_TYPE,&tmp_sensor_type);
if(isnan(tmp_sensor_type)==0) 	rangefinder.sensor_type=tmp_sensor_type;				
else rangefinder.sensor_type=rangefinder_type_default;
rangefinder.sensor_init_type=rangefinder.sensor_type;
switch(rangefinder.sensor_type)
{

}
}	


/***************************************
������:	void rangefinder_statemachine(void)
˵��: ���ɼ�״̬��
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void rangefinder_statemachine(void)
{
if(rangefinder.sensor_init_type!=rangefinder.sensor_type)//��ʼ��ʱ�����������뵱ǰ���ô��������Ͳ�һ��
{
	return;
	//������λ����Ч
}
switch(rangefinder.sensor_type)
{
	case 0 :
	case 1 :us100_statemachine();break;
	default:us100_statemachine();
}
}	


