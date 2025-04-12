#include "ti_msp_dl_config.h"
#include "datatype.h"
#include "system.h"
#include "user.h"
#include "nqei.h"
#include "stdio.h"


encoder NEncoder={
	.left_motor_period_ms=20,
	.right_motor_period_ms=20,
};
systime timer_qei1,timer_qei0;

extern motor_config trackless_motor;


///***************************************
//函数名:	void QEI0_IRQHandler(void)
//说明: QEI0中断服务函数
//入口:	无
//出口:	无
//备注:	无
//作者:	无名创新
//***************************************/
//void QEI0_IRQHandler(void)
//{
//	//NEncoder.right_motor_dir=((DL_GPIO_readPins(PORTB_PORT,  PORTB_RIGHT_DIR_PIN))!=0?1:-1);
//	NEncoder.right_motor_dir=((PORTB_PORT->DIN31_0 & PORTB_RIGHT_DIR_PIN)!=0?1:-1);
//	if(trackless_motor.right_encoder_dir_config==0)
//	{		
//		if(NEncoder.right_motor_dir==1)	NEncoder.right_motor_period_cnt++;
//		else if(NEncoder.right_motor_dir==-1) NEncoder.right_motor_period_cnt--;
//		//NEncoder.right_motor_total_cnt+=NEncoder.right_motor_dir;	
//	}
//	else
//	{
//		if(NEncoder.right_motor_dir==1)	NEncoder.right_motor_period_cnt++;
//			//NEncoder.right_motor_period_cnt++;
//		
//			//NEncoder.right_motor_period_cnt--;
//		
//		else if(NEncoder.right_motor_dir==-1)NEncoder.right_motor_period_cnt--;
//		//NEncoder.right_motor_total_cnt-=NEncoder.right_motor_dir;				
//	}	
//}

///***************************************
//函数名:	void QEI1_IRQHandler(void)
//说明: QEI1中断服务函数
//入口:	无
//出口:	无
//备注:	无
//作者:	无名创新
//***************************************/
//void QEI1_IRQHandler(void)
//{
//	//NEncoder.left_motor_dir=((DL_GPIO_readPins(PORTB_PORT,  PORTB_LEFT_DIR_PIN))!=0?-1:1);
//	NEncoder.left_motor_dir=((PORTB_PORT->DIN31_0 & PORTB_LEFT_DIR_PIN)!=0?-1:1);
//	//if(trackless_motor.left_encoder_dir_config==0)
//		if(PORTB_LEFT_DIR_PIN==1)
//	{	
//		if(NEncoder.left_motor_dir==1)	NEncoder.left_motor_period_cnt++;
//		else if(NEncoder.left_motor_dir==-1) NEncoder.left_motor_period_cnt--;
//		//NEncoder.left_motor_total_cnt+=NEncoder.left_motor_dir;
//	}
//	else if(PORTB_LEFT_DIR_PIN==-1)
//	{
//		if(NEncoder.left_motor_dir==1)	NEncoder.left_motor_period_cnt++;
//		else  if(NEncoder.left_motor_dir==-1) NEncoder.left_motor_period_cnt--;
//		//NEncoder.left_motor_total_cnt-=NEncoder.left_motor_dir;
//	}		
//}



int16 left_gpio_dianping;
int16 right_gpio_dianping;
int16 left_cnt;
int16 right_cnt;
float temp;




uint32_t gpio_interrup;

float Get_Encoder_countA1;
int16 Get_Encoder_countB2;

/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
//void GROUP1_IRQHandler1(void)
//void QEI0_IRQHandler(void)
//{
//// a26//4  //a17//5
//	DL_GPIO_getEnabledInterruptStatus(GPIOB,PORTB_LEFT_DIR_PIN|PORTB_LEFT_PULSE_PIN|PORTB_RIGHT_DIR_PIN|PORTB_RIGHT_PULSE_PIN);
//	//encoderA
//	
//	
//	
//				if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_PULSE_PIN)&&DL_GPIO_readPins(GPIOB,PORTB_RIGHT_DIR_PIN)) Get_Encoder_countA1++;
//        else Get_Encoder_countA1--;
//			
//	
//	
//	
//	
//	
//	
//	
//	
////	if((gpio_interrup & PORTB_RIGHT_DIR_PIN)==PORTB_RIGHT_DIR_PIN)
////	{
////		if(!DL_GPIO_readPins(GPIOB,PORTB_RIGHT_PULSE_PIN))
////		{
////			Get_Encoder_countA1--;
////		}
////		else
////		{
////			Get_Encoder_countA1++;
////		}
////	}
////	else if((gpio_interrup & PORTB_RIGHT_PULSE_PIN)==PORTB_RIGHT_PULSE_PIN)
////	{
////		if(!DL_GPIO_readPins(GPIOB,PORTB_RIGHT_DIR_PIN))
////		{
////			Get_Encoder_countA1++;
////		}
////		else
////		{
////			Get_Encoder_countA1--;
////		}
////	}
////	//encoderB
////	if((gpio_interrup & PORTB_LEFT_DIR_PIN)==PORTB_LEFT_DIR_PIN)
////	{
////		if(!DL_GPIO_readPins(GPIOB,PORTB_LEFT_PULSE_PIN))
////		{
////			Get_Encoder_countB2--;
////		}
////		else
////		{
////			Get_Encoder_countB2++;
////		}
////	}
////	else if((gpio_interrup & PORTB_LEFT_PULSE_PIN)==PORTB_LEFT_PULSE_PIN)
////	{
////		if(!DL_GPIO_readPins(GPIOB,PORTB_LEFT_DIR_PIN))
////		{
////			Get_Encoder_countB2++;
////		}
////		else
////		{
////			Get_Encoder_countB2--;
////		}
////	}
//	DL_GPIO_clearInterruptStatus(GPIOB,PORTB_LEFT_DIR_PIN|PORTB_LEFT_PULSE_PIN|PORTB_RIGHT_DIR_PIN|PORTB_RIGHT_PULSE_PIN);
//}





///***************************************
//???:	void QEI0_IRQHandler(void)
//??: QEI0??????
//??:	?
//??:	?
//??:	?
//??:	????
//***************************************/
//void QEI0_IRQHandler(void)
//{
//	//NEncoder.right_motor_dir=((DL_GPIO_readPins(PORTB_PORT,  PORTB_RIGHT_DIR_PIN))!=0?1:-1);
//	NEncoder.right_motor_dir=((PORTB_PORT->DIN31_0 & PORTB_RIGHT_DIR_PIN)!=0?1:-1);
//	if(trackless_motor.right_encoder_dir_config==0)
//	{		
//		if(NEncoder.right_motor_dir==1)	NEncoder.right_motor_period_cnt++;
//		else NEncoder.right_motor_period_cnt--;
//		NEncoder.right_motor_total_cnt+=NEncoder.right_motor_dir;	
//	}
//	else
//	{
//		if(NEncoder.right_motor_dir==1)	NEncoder.right_motor_period_cnt--;
//		else NEncoder.right_motor_period_cnt++;
//		NEncoder.right_motor_total_cnt-=NEncoder.right_motor_dir;				
//	}	
//}

///***************************************
//???:	void QEI1_IRQHandler(void)
//??: QEI1??????
//??:	?
//??:	?
//??:	?
//??:	????
//***************************************/
//void QEI1_IRQHandler(void)
//{
//	//NEncoder.left_motor_dir=((DL_GPIO_readPins(PORTB_PORT,  PORTB_LEFT_DIR_PIN))!=0?-1:1);
//	NEncoder.left_motor_dir=((PORTB_PORT->DIN31_0 & PORTB_LEFT_DIR_PIN)!=0?-1:1);
//	if(trackless_motor.left_encoder_dir_config==0)
//	{	
//		if(NEncoder.left_motor_dir==1)	NEncoder.left_motor_period_cnt++;
//		else NEncoder.left_motor_period_cnt--;
//		NEncoder.left_motor_total_cnt+=NEncoder.left_motor_dir;
//	}
//	else
//	{
//		if(NEncoder.left_motor_dir==1)	NEncoder.left_motor_period_cnt--;
//		else NEncoder.left_motor_period_cnt++;
//		NEncoder.left_motor_total_cnt-=NEncoder.left_motor_dir;
//	}		
//}


//#define PORTB_INT_IRQN                                          (GPIOB_INT_IRQn)
//#define PORTB_INT_IIDX               l          (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
//#define PORTB_RIGHT_PULSE_IIDX                               (DL_GPIO_IIDX_DIO4)
//#define PORTB_RIGHT_PULSE_PIN                                    (DL_GPIO_PIN_4)
//#define PORTB_RIGHT_PULSE_IOMUX                                  (IOMUX_PINCM17)
///* Defines for LEFT_PULSE: GPIOB.5 with pinCMx 18 on package pin 53 */
//#define PORTB_LEFT_PULSE_IIDX                                (DL_GPIO_IIDX_DIO5)
//#define PORTB_LEFT_PULSE_PIN            z                         (DL_GPIO_PIN_5)
//#define PORTB_LEFT_PULSE_IOMUX                                   (IOMUX_PINCM18)
///* Defines for RIGHT_DIR: GPIOB.6 with pinCMx 23 on package pin 58 */
//#define PORTB_RIGHT_DIR_PIN              l                        (DL_GPIO_PIN_6)
//#define PORTB_RIGHT_DIR_IOMUX                                    (IOMUX_PINCM23)
///* Defines for LEFT_DIR: GPIOB.7 with pinCMx 24 on package pin 59 */
//#define PORTB_LEFT_DIR_PIN            z                           (DL_GPIO_PIN_7)
//#define PORTB_LEFT_DIR_IOMUX                                     (IOMUX_PINCM24)



//#define PORTB_INT_IRQN                                          (GPIOB_INT_IRQn)
//#define PORTB_INT_IIDX                          (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
//#define PORTB_RIGHT_PULSE_IIDX                               (DL_GPIO_IIDX_DIO4)
//#define PORTB_RIGHT_PULSE_PIN                                    (DL_GPIO_PIN_4)
//#define PORTB_RIGHT_PULSE_IOMUX                                  (IOMUX_PINCM17)
///* Defines for LEFT_PULSE: GPIOB.5 with pinCMx 18 on package pin 53 */
//#define PORTB_LEFT_PULSE_IIDX                                (DL_GPIO_IIDX_DIO5)
//#define PORTB_LEFT_PULSE_PIN                                     (DL_GPIO_PIN_5)
//#define PORTB_LEFT_PULSE_IOMUX                                   (IOMUX_PINCM18)
///* Defines for RIGHT_DIR: GPIOB.6 with pinCMx 23 on package pin 58 */
//#define PORTB_left2_PULSE_IIDX                                (DL_GPIO_IIDX_DIO6)
//#define PORTB_RIGHT_DIR_PIN                                      (DL_GPIO_PIN_6)
//#define PORTB_RIGHT_DIR_IOMUX                                    (IOMUX_PINCM23)
///* Defines for LEFT_DIR: GPIOB.7 with pinCMx 24 on package pin 59 */
//#define PORTB_Right2_PULSE_IIDX                                (DL_GPIO_IIDX_DIO7)
//#define PORTB_LEFT_DIR_PIN                                       (DL_GPIO_PIN_7)
//#define PORTB_LEFT_DIR_IOMUX                                     (IOMUX_PINCM24)

uint32_t cnt=0;
int32_t _encoder_l_count = 0;
int32_t _encoder_R_count = 0;

uint8_t x,y,z,sum;
uint8_t xR,yR,zR,sumR;

void QEI0_IRQHandler(void)
{
	if(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) 
	{
		if( PORTB_INT_IIDX)
		{
			
		if(DL_GPIO_getEnabledInterruptStatus(GPIOB, PORTB_RIGHT_PULSE_PIN))
		{
			x=1;//A中断，则x=1;B中断，则x=0;
			
			//中断发生时，A相高电平，y=1；反之y=0;
			if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_PULSE_PIN)) y=1;
			else y=0;
			//中断发生时，B相高电平，z=1；反之z=0;
			if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_DIR_PIN)) z=1;
			else z=0;
			
			sum=x+y+z;//求和判断转动方向，偶数正转，奇数反转
			if(sum==0||sum==2) _encoder_l_count++;
			else _encoder_l_count--;
			Get_Encoder_countA1=_encoder_l_count;
			DL_GPIO_clearInterruptStatus(GPIOB, PORTB_RIGHT_PULSE_PIN);
		}
		
		if(DL_GPIO_getEnabledInterruptStatus(GPIOB, PORTB_RIGHT_DIR_PIN))
		{
			x=0;//A中断，则x=1;B中断，则x=0;
			//中断发生时，A相高电平，y=1；反之y=0;
			if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_PULSE_PIN)) y=1;
			else y=0;
			//中断发生时，B相高电平，z=1；反之z=0;
			if(DL_GPIO_readPins(GPIOA,PORTB_RIGHT_DIR_PIN)) z=1;
			else z=0;

			sum=x+y+z;//求和判断转动方向，偶数正转，奇数反转
			if(sum==0||sum==2) _encoder_l_count++;
			else _encoder_l_count--;
			Get_Encoder_countA1=_encoder_l_count;
			DL_GPIO_clearInterruptStatus(GPIOB, PORTB_RIGHT_DIR_PIN);
		}
		
		
		
		
		
		
		
				if(DL_GPIO_getEnabledInterruptStatus(GPIOB, PORTB_LEFT_PULSE_PIN))
		{
			xR=1;//A中断，则x=1;B中断，则x=0;
			
			//中断发生时，A相高电平，y=1；反之y=0;
			if(DL_GPIO_readPins(GPIOB,PORTB_LEFT_PULSE_PIN)) yR=1;
			else yR=0;
			//中断发生时，B相高电平，z=1；反之z=0;
			if(DL_GPIO_readPins(GPIOB,PORTB_LEFT_DIR_PIN)) zR=1;
			else zR=0;
			
			sumR=xR+yR+zR;//求和判断转动方向，偶数正转，奇数反转
			if(sumR==0||sumR==2) _encoder_R_count--;
			else _encoder_R_count++;
			Get_Encoder_countB2=_encoder_R_count;////////////////////////////////////////////////////
			DL_GPIO_clearInterruptStatus(GPIOB, PORTB_LEFT_PULSE_PIN);
		}
		
		if(DL_GPIO_getEnabledInterruptStatus(GPIOB, PORTB_LEFT_DIR_PIN))
		{
			xR=0;//A中断，则x=1;B中断，则x=0;
			//中断发生时，A相高电平，y=1；反之y=0;
			if(DL_GPIO_readPins(GPIOB,PORTB_LEFT_PULSE_PIN)) yR=1;
			else yR=0;
			//中断发生时，B相高电平，z=1；反之z=0;
			if(DL_GPIO_readPins(GPIOA,PORTB_LEFT_DIR_PIN)) zR=1;
			else zR=0;

			sumR=xR+yR+zR;//求和判断转动方向，偶数正转，奇数反转
			if(sumR==0||sum==2) _encoder_R_count--;
			else _encoder_R_count++;
			Get_Encoder_countB2=_encoder_R_count;//////////////////////////////////////////
			DL_GPIO_clearInterruptStatus(GPIOB, PORTB_LEFT_DIR_PIN);
		}
		

				
	}
	}
	
	
	
//	
//	
//		switch(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) 
//	{
//		case PORTB_INT_IIDX:
//		if(DL_GPIO_getEnabledInterruptStatus(GPIOB, PORTB_RIGHT_PULSE_PIN))
//		{
//			x=1;//A中断，则x=1;B中断，则x=0;
//			
//			//中断发生时，A相高电平，y=1；反之y=0;
//			if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_PULSE_PIN)) y=1;
//			else y=0;
//			//中断发生时，B相高电平，z=1；反之z=0;
//			if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_DIR_PIN)) z=1;
//			else z=0;
//			
//			sum=x+y+z;//求和判断转动方向，偶数正转，奇数反转
//			if(sum==0||sum==2) _encoder_l_count++;
//			else _encoder_l_count--;
//			Get_Encoder_countA1=_encoder_l_count*0.001;
//			DL_GPIO_clearInterruptStatus(GPIOB, PORTB_RIGHT_PULSE_PIN);
//		}
//		
//		if(DL_GPIO_getEnabledInterruptStatus(GPIOB, PORTB_RIGHT_DIR_PIN))
//		{
//			x=0;//A中断，则x=1;B中断，则x=0;
//			//中断发生时，A相高电平，y=1；反之y=0;
//			if(DL_GPIO_readPins(GPIOB,PORTB_RIGHT_PULSE_PIN)) y=1;
//			else y=0;
//			//中断发生时，B相高电平，z=1；反之z=0;
//			if(DL_GPIO_readPins(GPIOA,PORTB_RIGHT_DIR_PIN)) z=1;
//			else z=0;

//			sum=x+y+z;//求和判断转动方向，偶数正转，奇数反转
//			if(sum==0||sum==2) _encoder_l_count++;
//			else _encoder_l_count--;
//			Get_Encoder_countA1=_encoder_l_count*0.001;
//			DL_GPIO_clearInterruptStatus(GPIOB, PORTB_RIGHT_DIR_PIN);
//		}
//				
//		break;
//	}
	
	
	
	
	
	
	
	
	
	
	
}





































//void QEI0_IRQHandler(void)
//{
////	NEncoder.right_motor_dir=((DL_GPIO_readPins(PORTB_PORT,  PORTB_RIGHT_DIR_PIN))!=0?1:-1);
//	NEncoder.right_motor_dir=((PORTB_PORT->DIN31_0 & PORTB_RIGHT_DIR_PIN)!=0?1:-1);
////	if(PORTB_RIGHT_DIR_PIN==1)
////	{
////		NEncoder.right_motor_dir=1;
////		
////	}
////	else{
////		
////		
////		NEncoder.right_motor_dir=0;
////		
////	}
//	
//	
//	
//	if(trackless_motor.right_encoder_dir_config==0)
//	{		
//		NEncoder.right_motor_dir=DL_GPIO_readPins(PORTB_PORT,  PORTB_RIGHT_DIR_PIN);
//	if(NEncoder.right_motor_dir==64||NEncoder.right_motor_dir==-64)NEncoder.right_motor_dir=1;
//		if(NEncoder.right_motor_dir==1)	NEncoder.right_motor_period_cnt++;
//		else NEncoder.right_motor_period_cnt--;
//	//	NEncoder.right_motor_total_cnt+=NEncoder.right_motor_dir;	
//	}
//	else
//	{
//		if(NEncoder.right_motor_dir==1)	NEncoder.right_motor_period_cnt--;
//		else NEncoder.right_motor_period_cnt++;
//		NEncoder.right_motor_total_cnt-=NEncoder.right_motor_dir;				
//	}	
//}


//void QEI1_IRQHandler(void)
//{
////	NEncoder.left_motor_dir=((DL_GPIO_readPins(PORTB_PORT,  PORTB_LEFT_DIR_PIN))!=0?-1:1);
//	NEncoder.left_motor_dir=((PORTB_PORT->DIN31_0 & PORTB_LEFT_DIR_PIN)!=0?-1:1);
//	if(trackless_motor.left_encoder_dir_config==0)
//	{	
//	
////	left_gpio_dianping=DL_GPIO_readPins(PORTB_PORT,  PORTB_LEFT_DIR_PIN);
////	
////	
////			if(left_gpio_dianping==1)	left_cnt++;
////	    else left_cnt--;
//	
//	
//	NEncoder.left_motor_dir=DL_GPIO_readPins(PORTB_PORT,  PORTB_LEFT_DIR_PIN);
//	
////	if(NEncoder.left_motor_dir==128||NEncoder.left_motor_dir==-128)NEncoder.left_motor_dir=0;
//		if(NEncoder.left_motor_dir==1)	NEncoder.left_motor_period_cnt++;
//		else NEncoder.left_motor_period_cnt--;
//		//NEncoder.left_motor_total_cnt+=NEncoder.left_motor_dir;
//	}
//	else
//	{
//		if(NEncoder.left_motor_dir==1)	NEncoder.left_motor_period_cnt--;
//		else NEncoder.left_motor_period_cnt++;
//		NEncoder.left_motor_total_cnt-=NEncoder.left_motor_dir;
//	}		
//}
/***************************************
函数名:	void get_left_motor_speed(void)
说明: 获取左边轮子实际速度值
入口:	无
出口:	无
备注:	将单位时间内的脉冲数,转化成rpm、cm/s
作者:	无名创新
***************************************/
float get_left_motor_speed(void)
{
	static uint16_t cnt1=0;cnt1++;
	if(cnt1>=4)
	{
		
		cnt1=0;
		NEncoder.left_motor_period_ms=20;
		//将速度转化成转每分钟
//		NEncoder.left_motor_speed_rpm=60*(NEncoder.left_motor_period_cnt*1.0f/trackless_motor.pulse_num_per_circle)
//																	/(NEncoder.left_motor_period_ms*0.001f);	
//		NEncoder.left_motor_speed_cmps=2*3.14f*trackless_motor.wheel_radius_cm*(NEncoder.left_motor_speed_rpm/60.0f);
		
		//NEncoder.left_motor_speed_cmps=NEncoder.left_motor_period_cnt;
		NEncoder.left_motor_speed_cmps=_encoder_l_count;
		Get_Encoder_countA1=0;
	
		//NEncoder.left_motor_period_cnt=0;
		_encoder_l_count=0;

	}
	return NEncoder.left_motor_speed_cmps;
//	return NEncoder.left_motor_speed_cmps;
}

/***************************************
函数名:	void get_right_motor_speed(void)
说明: 获取右边轮子实际速度值
入口:	无
出口:	无
备注:	将单位时间内的脉冲数,转化成rpm、cm/s
作者:	无名创新
***************************************/
float get_right_motor_speed(void)
{
	temp=0;
	static uint16_t cnt2=0;cnt2++;
	if(cnt2>=4)
	{
		//float temp;
		cnt2=0;
		NEncoder.right_motor_period_ms=20;
//		//将速度转化成转每分钟
////		NEncoder.right_motor_speed_rpm=60*(NEncoder.right_motor_period_cnt*1.0f/trackless_motor.pulse_num_per_circle)
////																	/(NEncoder.right_motor_period_ms*0.001f);
////		NEncoder.right_motor_speed_cmps=2*3.14f*trackless_motor.wheel_radius_cm*(NEncoder.right_motor_speed_rpm/60);
//	//	NEncoder.right_motor_speed_cmps=NEncoder.right_motor_speed_rpm=NEncoder.right_motor_period_cnt;
//				NEncoder.right_motor_speed_rpm=60*(Get_Encoder_countA1*1.0f/trackless_motor.pulse_num_per_circle)
//																	/(NEncoder.right_motor_period_ms*0.001f);
//		  NEncoder.right_motor_speed_cmps=2*3.14f*trackless_motor.wheel_radius_cm*(NEncoder.right_motor_speed_rpm/60);
	//	temp=Get_Encoder_countA1;
		//Get_Encoder_countA1=0;
		NEncoder.right_motor_speed_cmps=Get_Encoder_countB2;
		_encoder_R_count=0;
		Get_Encoder_countB2=0;	
	}
	return NEncoder.right_motor_speed_cmps;
//	 return Get_Encoder_countA1;
}

/***************************************
函数名:	void motor_total_cnt_reset(void)
说明: 总脉冲计数复位
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void motor_total_cnt_reset(void)
{
	NEncoder.left_motor_total_cnt =0;
	NEncoder.right_motor_total_cnt=0;
}


/***************************************
函数名:	void Encoder_Init(void)
说明: 编码器采集初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void Encoder_Init(void)
{

	
	
	
	
}


