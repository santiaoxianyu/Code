#include "headfile.h"
#include "gray_detection.h"
#include "user.h"


#define steer_deadzone 50//50  ת���������

typedef struct
{
	uint16_t state;
  bool gray_bit[16];
}_gray_state; 
_gray_state gray_state;
float gray_status[2]={0},gray_status_backup[2][20]={0};//�Ҷȴ�����״̬����ʷֵ
float turn_output=0,turn_output_last=0;//���������ֵ
float	turn_scale=turn_scale_default;//ת����Ʋ���ϵ��  0.15
uint32_t gray_status_worse=0;	//�Ҷȹ��쳣״̬������
controller seektrack_ctrl[2];		//����Ѱ���������ṹ��
uint32_t vision_status_worse=0;



/***************************************************
������: void gpio_input_init(void)
˵��:	12·�Ҷȹ�gpio����ʼ��
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/


void gpio_input_init(void)
{

}

#define read_gray_bit1   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT0_PIN ) ? 0x01 : 0x00)
#define read_gray_bit2   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT1_PIN ) ? 0x01 : 0x00)
#define read_gray_bit3   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT2_PIN ) ? 0x01 : 0x00)
#define read_gray_bit4   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT3_PIN ) ? 0x01 : 0x00)
#define read_gray_bit5   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT4_PIN ) ? 0x01 : 0x00)
#define read_gray_bit6   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT5_PIN ) ? 0x01 : 0x00)
#define read_gray_bit7   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT6_PIN ) ? 0x01 : 0x00)
#define read_gray_bit8   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT7_PIN ) ? 0x01 : 0x00)
#define read_gray_bit9   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT8_PIN ) ? 0x01 : 0x00)
#define read_gray_bit10  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT9_PIN ) ? 0x01 : 0x00)
#define read_gray_bit11  ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT10_PIN) ? 0x01 : 0x00)
#define read_gray_bit12  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT11_PIN) ? 0x01 : 0x00)


/***************************************************
������: void gpio_input_check_channel_7(void)
˵��:	7·�Ҷȹ�gpio���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void gpio_input_check_channel_7(void)
{
	gray_state.gray_bit[0]=read_gray_bit1;
  gray_state.gray_bit[1]=read_gray_bit2;
	gray_state.gray_bit[2]=read_gray_bit3;
	gray_state.gray_bit[3]=read_gray_bit4;
	
  gray_state.gray_bit[4]=read_gray_bit5;
	gray_state.gray_bit[5]=read_gray_bit6;
	gray_state.gray_bit[6]=read_gray_bit7;
	gray_state.state=0x0000;	
	for(uint16_t i=0;i<6;i++)
	{
		gray_state.state|=gray_state.gray_bit[i]<<i;
	}
	
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[0][i]=gray_status_backup[0][i-1];
	}
	gray_status_backup[0][0]=gray_status[0];
	switch(gray_state.state)
	{
		case 0x0001:gray_status[0]=-7;	gray_status_worse/=2;break;									//0000001b
		case 0x0003:gray_status[0]=-5;	gray_status_worse/=2;break;									//0000011b
	  case 0x0002:gray_status[0]=-4;	gray_status_worse/=2;break;									//0000010b
		case 0x0006:gray_status[0]=-3;	gray_status_worse/=2;break;									//0000110b
		case 0x0004:gray_status[0]=-1;	gray_status_worse/=2;break;									//0000100b
		case 0x000C:gray_status[0]=0 ;	gray_status_worse/=2;break;									//0001100b
		case 0x0008:gray_status[0]=0;	  gray_status_worse/=2;break;									//0001000b
		case 0x0018:gray_status[0]=0;   gray_status_worse/=2;break;									//0011000b
		case 0x0010:gray_status[0]=1;   gray_status_worse/=2;break;									//0010000b
		case 0x0030:gray_status[0]=3;   gray_status_worse/=2;break;									//0110000b
		case 0x0020:gray_status[0]=4;   gray_status_worse/=2;break;									//0100000b
		case 0x0060:gray_status[0]=5;   gray_status_worse/=2;break;									//1100000b
		case 0x0040:gray_status[0]=7;   gray_status_worse/=2;break;									//1000000b
		case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break; //0000000b
		default:
		{
			gray_status[0]=gray_status_backup[0][0];
			gray_status_worse++;
		}
	}
}


/***************************************************
������: void gpio_input_check_channel_12(void)
˵��:	12·�Ҷȹ�gpio���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void gpio_input_check_channel_12(void)
{
	gray_state.gray_bit[0]=read_gray_bit1;
	gray_state.gray_bit[1]=read_gray_bit2;
	gray_state.gray_bit[2]=read_gray_bit3;
	gray_state.gray_bit[3]=read_gray_bit4;

	gray_state.gray_bit[4]=read_gray_bit5;
	gray_state.gray_bit[5]=read_gray_bit6;
	gray_state.gray_bit[6]=read_gray_bit7;
	gray_state.gray_bit[7]=read_gray_bit8;
	
	gray_state.gray_bit[8] =read_gray_bit9;
	gray_state.gray_bit[9] =read_gray_bit10;
	gray_state.gray_bit[10]=read_gray_bit11;
	gray_state.gray_bit[11]=read_gray_bit12;
	gray_state.state=0x0000;	
	for(uint16_t i=0;i<11;i++)
	{
		gray_state.state|=gray_state.gray_bit[i]<<i;
	}	
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[0][i]=gray_status_backup[0][i-1];
	}
	gray_status_backup[0][0]=gray_status[0];
	switch(gray_state.state)
	{
		case 0x0001:gray_status[0]=-11; gray_status_worse/=2;break;									//000000000001b
		
		case 0x0003:gray_status[0]=-10; gray_status_worse/=2;break;									//000000000011b
	  case 0x0002:gray_status[0]=-9;	gray_status_worse/=2;break;									//000000000010b
		case 0x0006:gray_status[0]=-8;	gray_status_worse/=2;break;									//000000000110b
		
		case 0x0004:gray_status[0]=-7;	gray_status_worse/=2;break;									//000000000100b
		case 0x000C:gray_status[0]=-6;	gray_status_worse/=2;break;									//000000001100b
		case 0x0008:gray_status[0]=-5;	gray_status_worse/=2;break;									//000000001000b
		case 0x0018:gray_status[0]=-0;	gray_status_worse/=2;break;									//000000011000b
		
		case 0x0010:gray_status[0]=-3;	gray_status_worse/=2;break;									//000000010000b
		case 0x0030:gray_status[0]=-2;	gray_status_worse/=2;break;									//000000110000b
		case 0x0020:gray_status[0]=-1;	gray_status_worse/=2;break;									//000000100000b
		case 0x0060:gray_status[0]=0;		gray_status_worse/=2;break;									//000001100000b
		case 0x0040:gray_status[0]=1;		gray_status_worse/=2;break;									//000001000000b
		case 0x00C0:gray_status[0]=2;		gray_status_worse/=2;break;									//000011000000b
		case 0x0080:gray_status[0]=3;		gray_status_worse/=2;break;									//000010000000b
		case 0x0180:gray_status[0]=0;		gray_status_worse/=2;break;									//000110000000b
		case 0x0100:gray_status[0]=5;		gray_status_worse/=2;break;									//000100000000b
		case 0x0300:gray_status[0]=6;		gray_status_worse/=2;break;									//001100000000b
		case 0x0200:gray_status[0]=7;		gray_status_worse/=2;break;									//001000000000b
		
		case 0x0600:gray_status[0]=8;		gray_status_worse/=2;break;									//011000000000b
		case 0x0400:gray_status[0]=9;		gray_status_worse/=2;break;									//010000000000b
		case 0x0C00:gray_status[0]=10;	gray_status_worse/=2;break;									//110000000000b
		case 0x0800:gray_status[0]=11;	gray_status_worse/=2;break;									//100000000000b
		case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break; //00000000b
		default://����������������ж�
		{
			gray_status[0]=gray_status_backup[0][0];
			gray_status_worse++;
		}
	}	
	
//	static uint16_t tmp_cnt=0;
//	switch(gray_state.state)//ֹͣ�߼��
//	{
//		case 0x0030:tmp_cnt++;break;//000000110000b
//		case 0x0020:tmp_cnt++;break;//000000100000b
//		case 0x0060:tmp_cnt++;break;//000001100000b
//		case 0x0040:tmp_cnt++;break;//000001000000b
//		case 0x00C0:tmp_cnt++;break;//000011000000b
//		case 0x00F0://000011110000b
//		{
//			if(tmp_cnt>=10)
//			{
//				tmp_cnt=0;
//				beep.period=200;//200*5ms
//				beep.light_on_percent=0.5f;			
//				beep.reset=1;
//				beep.times=1;			
//			}
//		}
//		break;
//	}		
}


#define startpoint_check_state_keep_cm  50.0f
float startpoint_distance_cm=0;
uint8_t startpoint_check_flag=0;
uint8_t track_switch_flag=0;
float startpoint_straightaway_cm=0;
/***************************************************
������: void gpio_input_check_channel_12_with_handle(void)
˵��:	12·�Ҷȹ�gpio���,��������Ϣ���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
extern int huidu1,huidu2,huidu3,huidu4,huidu5,huidu6,huidu7,huidu8,huidu9,huidu10,huidu11,huidu12;

extern float err_aul_huidu;
	
void gpio_input_check_channel_12_with_handle(void)
{
	huidu1=gray_state.gray_bit[0]=read_gray_bit1;
//		huidu1=gray_state.gray_bit[0]=0;
	huidu2=gray_state.gray_bit[1]=read_gray_bit2;
	huidu3=gray_state.gray_bit[2]=read_gray_bit3;
   huidu4=gray_state.gray_bit[3]=read_gray_bit4;
	
	
	huidu5=gray_state.gray_bit[4]=0;//duicheng

//	huidu5=gray_state.gray_bit[4]=read_gray_bit5;
	huidu6=gray_state.gray_bit[5]=read_gray_bit6;
	huidu7=gray_state.gray_bit[6]=read_gray_bit7;
//	huidu8=gray_state.gray_bit[7]=read_gray_bit8;
	
		huidu8=gray_state.gray_bit[7]=0;//huaila
	
	huidu9=gray_state.gray_bit[8]=read_gray_bit9;
	huidu10=gray_state.gray_bit[9]=read_gray_bit10;
	huidu11=gray_state.gray_bit[10]=read_gray_bit11;
//	huidu12=gray_state.gray_bit[11]=0;
	huidu12=gray_state.gray_bit[11]=read_gray_bit12;
	
	gray_state.state=0x0000;	
	for(uint16_t i=0;i<11;i++)
	{
		gray_state.state|=gray_state.gray_bit[i]<<i;
	}	
	
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[0][i]=gray_status_backup[0][i-1];
	}
	gray_status_backup[0][0]=gray_status[0];
	switch(gray_state.state)
	{
		case 0x0001:gray_status[0]=err_aul_huidu=-8;gray_status_worse/=2;break;									//000000000001b
		case 0x0003:gray_status[0]=err_aul_huidu=-8;gray_status_worse/=2;break;									//000000000011b
	  case 0x0002:gray_status[0]=err_aul_huidu=-8;	gray_status_worse/=2;break;									//000000000010b
		case 0x0006:gray_status[0]=err_aul_huidu=-7;	gray_status_worse/=2;break;									//000000000110b
		case 0x0004:gray_status[0]=err_aul_huidu=-7;	gray_status_worse/=2;break;									//000000000100b
		case 0x000C:gray_status[0]=err_aul_huidu=-5;	gray_status_worse/=2;break;									//000000001100b
		case 0x0008:gray_status[0]=err_aul_huidu=-4;	gray_status_worse/=2;break;									//000000001000b
		case 0x0018:gray_status[0]=err_aul_huidu=-4;	gray_status_worse/=2;break;									//000000011000b
		case 0x0010:gray_status[0]=err_aul_huidu=-3;	gray_status_worse/=2;break;									//000000010000b
		case 0x0030:gray_status[0]=err_aul_huidu=-2;	gray_status_worse/=2;break;									//000000110000b
		case 0x0020:gray_status[0]=err_aul_huidu=-0;	gray_status_worse/=2;break;									//000000100000b
		case 0x0060:gray_status[0]=err_aul_huidu=0;		gray_status_worse/=2;break;									//000001100000b
		case 0x0040:gray_status[0]=err_aul_huidu=0;		gray_status_worse/=2;break;									//000001000000b
		case 0x00C0:gray_status[0]=err_aul_huidu=2;		gray_status_worse/=2;break;									//000011000000b
		case 0x0080:gray_status[0]=err_aul_huidu=3;		gray_status_worse/=2;break;									//000010000000b
		case 0x0180:gray_status[0]=err_aul_huidu=4;		gray_status_worse/=2;break;									//000110000000b
		case 0x0100:gray_status[0]=err_aul_huidu=4;		gray_status_worse/=2;break;									//000100000000b
		case 0x0300:gray_status[0]=err_aul_huidu=5;		gray_status_worse/=2;break;									//001100000000b
		case 0x0200:gray_status[0]=err_aul_huidu=7;		gray_status_worse/=2;break;									//001000000000b
		case 0x0600:gray_status[0]=err_aul_huidu=7;		gray_status_worse/=2;break;									//011000000000b
		case 0x0400:gray_status[0]=err_aul_huidu=8;		gray_status_worse/=2;break;									//010000000000b
		case 0x0C00:gray_status[0]=err_aul_huidu=8;	gray_status_worse/=2;break;									//110000000000b
		case 0x0800:gray_status[0]=err_aul_huidu=8;	gray_status_worse/=2;break;									//100000000000b
		//case 0x0000:gray_status[0]=gray_status_backup[0][0];gray_status_worse++;break;//000000000000b
//		default://����������������ж�
//		{
//			uint16_t tmp_state=gray_state.state;
//			if(startpoint_check_flag==1&&track_switch_flag==1)//�л���Ȧ
//			{
//				//ֻ��Ӧ���Ҷȹܵ�״̬
//				//tmp_state&=0x0FC0;//&&111111000000
//				tmp_state&=0x0F80;//&&111110000000
//				switch(tmp_state)
//				{
//					case 0x0040:gray_status[0]=1;	break;//000001000000b
//					case 0x00C0:gray_status[0]=2;	break;//000011000000b
//					case 0x0080:gray_status[0]=3;	break;//000010000000b
//					case 0x0180:gray_status[0]=4;	break;//000110000000b
//					case 0x0100:gray_status[0]=5;	break;//000100000000b
//					case 0x0300:gray_status[0]=6;	break;//001100000000b
//					case 0x0200:gray_status[0]=7;	break;//001000000000b
//					case 0x0600:gray_status[0]=8;	break;//011000000000b
//					case 0x0400:gray_status[0]=9;	break;//010000000000b
//					case 0x0C00:gray_status[0]=10;break;//110000000000b
//					case 0x0800:gray_status[0]=11;break;//100000000000b				
//					default:gray_status[0]=0;
//				}
//			}
//			else if(startpoint_check_flag==1&&track_switch_flag==0)//�л���Ȧ
//			{
//				//ֻ��Ӧ�м�+�Ҳ��Ҷȹܵ�״̬
//				tmp_state&=0x00FF;//&&000011111111
//				switch(tmp_state)
//				{
//					case 0x0001:gray_status[0]=-11;break;//000000000001b
//					case 0x0003:gray_status[0]=-10;break;//000000000011b
//					case 0x0002:gray_status[0]=-9; break;//000000000010b
//					case 0x0006:gray_status[0]=-8; break;//000000000110b
//					case 0x0004:gray_status[0]=-7; break;//000000000100b
//					case 0x000C:gray_status[0]=-6; break;//000000001100b
//					case 0x0008:gray_status[0]=-5; break;//000000001000b
//					case 0x0018:gray_status[0]=-4; break;//000000011000b
//					case 0x0010:gray_status[0]=-3; break;//000000010000b
//					case 0x0030:gray_status[0]=-2; break;//000000110000b	
//					case 0x0020:gray_status[0]=-1;break;//000000100000b
//					case 0x0060:gray_status[0]=0;	break;//000001100000b
//					case 0x0040:gray_status[0]=1;	break;//000001000000b
//					case 0x00C0:gray_status[0]=2;	break;//000011000000b		
//					default:gray_status[0]=0;

//				}				
//			}
//			else
//			{
//				gray_status[0]=gray_status_backup[0][0];
//				gray_status_worse++;			
//			}
//		}
//	}	
//	float pos_delta=smartcar_imu.state_estimation.speed*0.001f;
//	switch(gray_state.state)//�����������⴦��
//	{
//		case 0x0001:startpoint_straightaway_cm=0;break;									//000000000001b
//		case 0x0003:startpoint_straightaway_cm=0;break;									//000000000011b
//	  case 0x0002:startpoint_straightaway_cm=0;break;									//000000000010b
//		case 0x0006:startpoint_straightaway_cm=0;break;									//000000000110b
//		case 0x0004:startpoint_straightaway_cm=0;break;									//000000000100b
//		case 0x000C:startpoint_straightaway_cm=0;break;									//000000001100b
//		case 0x0008:startpoint_straightaway_cm=0;break;									//000000001000b
//		case 0x0018:startpoint_straightaway_cm=0;break;									//000000011000b
//		case 0x0010:startpoint_straightaway_cm=0;break;									//000000010000b
//		case 0x0030:startpoint_straightaway_cm+=pos_delta;break;				//000000110000b
//		case 0x0020:startpoint_straightaway_cm+=pos_delta;break;				//000000100000b
//		case 0x0060:startpoint_straightaway_cm+=pos_delta;break;				//000001100000b   ���м�
//		case 0x0040:startpoint_straightaway_cm+=pos_delta;break;				//000001000000b
//		case 0x00C0:startpoint_straightaway_cm+=pos_delta;break;				//000011000000b
//		case 0x0080:startpoint_straightaway_cm=0;break;									//000010000000b
//		case 0x0180:startpoint_straightaway_cm=0;break;									//000110000000b
//		case 0x0100:startpoint_straightaway_cm=0;break;									//000100000000b
//		case 0x0300:startpoint_straightaway_cm=0;break;									//001100000000b
//		case 0x0200:startpoint_straightaway_cm=0;break;									//001000000000b
//		case 0x0600:startpoint_straightaway_cm=0;break;									//011000000000b
//		case 0x0400:startpoint_straightaway_cm=0;break;									//010000000000b
//		case 0x0C00:startpoint_straightaway_cm=0;break;									//110000000000b
//		case 0x0800:startpoint_straightaway_cm=0;break;									//100000000000b
//		//�������α���������������Ҳ���Խ����Կ������м�������Լӣ��൱��ֱ��·Լ��
//		//case 0x00F0://000011110000b//������λ
//		//case 0x01E0://000111100000b//������λ
//		//case 0x0078://000001111000b//������λ
//		case 0x01F0://000111110000b//������λ
//		case 0x00F8://000011111000b//������λ
//		case 0x01F8://000111111000b//������λ
//		{
//			if(startpoint_check_flag==1)//�Ѽ�⵽ֹͣ�㣬��־λδ���ʱ��ǰ�˳�
//			{
//					break;
//			}
//			
//			//30cmֱ��Լ��
//			if(startpoint_straightaway_cm>=30)//����������ʱ,�ɳ��ԼӴ��ֵ
//			{
//				startpoint_straightaway_cm=0;
//				startpoint_check_flag=1;
//				startpoint_distance_cm=smartcar_imu.state_estimation.distance;//��¼ʶ�𵽵ĳ��������ڵľ���
//				
//				if(track_switch_flag==0) 
//				{
//					track_switch_flag=1;
//					bling_set(&light_red,2000,200,0.5,0,0);
//				}
//				else 
//				{
//					track_switch_flag=0;
//					bling_set(&light_blue,2000,200,0.5,0,0);
//				}
//				
//				beep.period=100;//200*5ms
//				beep.light_on_percent=0.5f;			
//				beep.reset=1;
//				beep.times=1;			
//			}
//		}
//		break;
//		default:
//		{
//			//startpoint_straightaway_cm=0;
//		}
//	}	

//	if(startpoint_check_flag==1)//��⵽ֹͣ��//no use
//	{
//		if(smartcar_imu.state_estimation.distance-startpoint_distance_cm>=startpoint_check_state_keep_cm)//����־λ���־���Ϊ50cm	
//		{
//			startpoint_check_flag=0;
//			beep.period=50;//200*5ms
//			beep.light_on_percent=0.5f;			
//			beep.reset=1;
//			beep.times=2;					
//		}

	}
	
//	if(err_aul_huidu>11)err_aul_huidu=err_aul_huidu*0.1;
//	if(err_aul_huidu<-11)err_aul_huidu=err_aul_huidu*0.1;
	//err_aul_huidu=gray_status[0];  
	
	
}



/***************************************************
������: void gray_turn_control_200hz(float *output)
˵��:	�Ҷȹ�Ѱ��ʱ��ת�����
���:	float *output-���������
����:	��
��ע:	��
����:	��������
****************************************************/
void gray_turn_control_200hz(float *output)
{
		//�����쳣����
//	if(gray_status_worse>=1000)//����1S�ڼ�ⲻ���������ߣ������϶�Ϊ�ܳ��������ߴ���
//	{
//		trackless_output.unlock_flag=LOCK;//����
//	}	// no use
	
	//�����ϴο��������
	turn_output_last=turn_output;//di tong lv bo
	//ת��PD����������������ʵʱ��Ҫ��ߣ��������I��ʹ������Ӧ�ͺ�
	seektrack_ctrl[0].expect=0;							//����--kanqingkuang 
	seektrack_ctrl[0].feedback=gray_status[0];	//����
	pid_control_run(&seektrack_ctrl[0]);		  //����������
	turn_output=seektrack_ctrl[0].output;
	//������������
	if(turn_output>0) turn_output+=steer_deadzone;
	if(turn_output<0) turn_output-=steer_deadzone;
	//����޷�
	turn_output=constrain_float(turn_output,-15,20);//ת���������޷� gei xiao dian
	
	*output=0.75f*turn_output+0.25f*turn_output_last;//���Ϊǰ�����μ���ľ�ֵ 
}



/***************************************************
������: void gpio_input_check_from_vision(void)
˵��:	openmv�Ӿ�Ѱ��ʱ��ƫ����
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void gpio_input_check_from_vision(void)
{
	static uint16_t cnt=0;
	cnt++;if(cnt<=20) return;cnt=0;//���ڿ���4*5ms=20ms
	
	//�Ӿ�����ƫ����50hz��Ƶ��������洢
	for(uint16_t i=19;i>0;i--)
	{
		gray_status_backup[1][i]=gray_status_backup[1][i-1];
	}
	gray_status_backup[1][0]=gray_status[1];

	//gray_status[1]=camera1.sdk_target.x;
}

/***************************************************
������: void vision_turn_control_50hz(float *output)
˵��:	openmv�Ӿ�Ѱ��ʱ��ת�����
���:	float *output-���������
����:	��
��ע:	��
����:	��������
****************************************************/
void vision_turn_control_50hz(float *output)
{
	static uint16_t cnt=0;
	cnt++;if(cnt<=4) return;cnt=0;//���ڿ���4*5ms=20ms
	
	//�����ϴο��������
	turn_output_last=turn_output;
	//ת��PD����������������ʵʱ��Ҫ��ߣ��������I��ʹ������Ӧ�ͺ�
	seektrack_ctrl[1].expect=0;							//����
	seektrack_ctrl[1].feedback=gray_status[1];	//����
	pid_control_run(&seektrack_ctrl[1]);		  //����������
	turn_output=seektrack_ctrl[1].output;
	//������������
	if(turn_output>0) turn_output+=0;
	if(turn_output<0) turn_output-=0;
	//����޷�
	turn_output=constrain_float(turn_output,-500,500);//ת���������޷�
	
	*output=turn_output;
}

