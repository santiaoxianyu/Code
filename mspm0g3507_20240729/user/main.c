/****************************************************************************************
	MSPM0G3507电赛小车开源方案资源分配表--MSPM0学习中心交流群828746221		
	功能	单片机端口	外设端口
	无名创新地面站通讯	
		PA10-->UART0-TXD	USB转TTL-RXD
		PA11-->UART0-RXD	USB转TTL-TXD
	机器视觉OPENMV4 MINI	
		PA8-UART1-TXD	UART3-RXD
		PA9-->UART1-RXD	UART3-TXD
	手机蓝牙APP地面站	
		PA21-UART2-TXD	蓝牙串口模块RXD
		PA22-->UART2-RXD	蓝牙串口模块TXD
	US100超声波模块	
		PB2-UART3-TXD	US100-TX/TRIG
		PB3-->UART3-RXD	US100-RX/ECHO
	12路灰度传感器FPC	
	  PA31-->P1
		PA28-->P2
		PA1-->P3
		PA0-->P4
		PA25-->P5
		PA24-->P6
		PB24-->P7
		PB23-->P8
		PB19-->P9
		PB18-->P10
		PA16-->P11
		PB13-->P12
	电机控制MPWM	
		PA4-A0-PWM-CH3	  右边电机调速INA1
		PA7-->A0-PWM-CH2	右边电机调速INA2
		PA3-->A0-PWM-CH1	左边电机调速INB1
		PB14-->A0-PWM-CH0	左边电机调速INB2		
	舵机控制SPWM	
		PA15-A1-PWM-CH0	  预留1
		PB1-->A1-PWM-CH1	预留2
		PA23-->G7-PWM-CH0	预留3
		PA2-->G7-PWM-CH1	前轮舵机转向控制PWM
	编码器测速ENC	
		PB4-RIGHT-PULSE	  右边电机脉冲倍频输出P1
		PB5-->LEFT-PULSE	左边电机脉冲倍频输出P2
		PB6-->RIGHT-DIR	  右边电机脉冲鉴相输出D1
		PB7-->LEFT-DIR	  左边电机脉冲鉴相输出D2
	外置IMU接口IMU	
		PA29-I2C-SCL	MPU6050-SCL
		PA30-->I2C-SDA	MPU6050-SDA
		PB0-->HEATER	温控IO可选
	电池电压采集	
		PA26-ADC-VBAT	需要外部分压后才允许接入
****************************************************************************************/

#include "ti_msp_dl_config.h"
#include "headfile.h"
#include "stdio.h"
short yaw;
short angle;

float speed_right,speed_left;

extern float Pitch,Roll,Yaw;

void steering_ring(void);//转向�?
void Track_ring(void);//寻迹�?
void one(void);
void two(void);
void three(void);
void four(void);
void Load(int16_t left_pwm1, int16_t right_pwm1);
//转向环变�?
int16_t flag;
int16_t straight_right;
int16_t straight_left;
int16_t pid_turn;
extern unsigned int  rawGyroZ;

//寻迹环变�?
int16_t track_turn,track_left,track_right;
//编码器�?�电�?
uint32_t sys_tick; 
int Encoder1,Encoder2;
int Encoder1_last,Encoder2_last;
int16_t left,right,left_last,right_last;
int16_t L_Target_Position, L_Target_Speed;
extern int8_t track_sum;
extern int8_t B1_put,B2_put,B3_put,B4_put;
//�?螺仪1
//float yaww,yawl,yaw_init,yaw_out;
//int Init_Angle_Flag=1;
//float q0, q1, q2, q3;
//float roll, pitch, yaw;
extern int8_t track_assignment[12];

//float gyro_Z ;
//板子上的电机2的PID控制�?
PID_Controller left_pid = {
    .p=160.0f,
    .i=18.0f
}; 
//板子上的电机1的PID控制�?
PID_Controller right_pid = {
    .p=160.0f,
    .i=18.0f
}; 
//直走pid控制�?
PID_Controller yaw_pid={
    .p=1.0f,
    .d=1.5f
  };
//寻迹pid控制�?  
PID_Controller track_pid_assignment={
//	.p=2.0f
    .p=3.0f,
    .d=5.5f,	
};


int main(void)
{
	
//	usart_irq_config();     //串口中断配置0x00001000
  SYSCFG_DL_init();	      //系统资源配置初始化	
	OLED_Init();						//显示屏初始化
	ctrl_params_init();			//控制参数初始化
	trackless_params_init();//硬件配置初始化
	simulation_pwm_init();  //模拟PWM初始化
	rgb_init();							//RGB灯初始化	
	Encoder_Init();					//编码器资源初始化
	Button_Init();					//板载按键初始化
	timer_irq_config();     //中断初始化  
	gpio_input_init();
	PPM_Init();							//编码器
	DL_GPIO_clearPins(PORTA_PORT,PORTA_BEEP_PIN);
	usart_irq_config();     //串口中断配置0x00001000
	int s;
  while(1)
  {

		s=Track_follow();
		DL_UART_Main_transmitDataBlocking(UART_1_INST,1);
		SendDataToVOFA(30,speed_left,speed_right);
		display_6_8_number(1 ,1,Yaw);
		display_6_8_number(1 ,2,Roll);
		display_6_8_number(1 ,3,Pitch);
		display_6_8_number(1 ,5,speed_right);
		display_6_8_number(1 ,6,s);
		
		if(B1_put==1){display_6_8_number(1,4,1);DL_GPIO_setPins(KEYB_PORT,PORTB_D3_5_PIN);}
		else if(B1_put==0){display_6_8_number(1,4,0);DL_GPIO_clearPins(KEYB_PORT,PORTB_D3_5_PIN);}
//		if(B2_put==1){display_6_8_number(8,4,2);}
//		else if(B2_put==0){display_6_8_number(8,4,0);}
//		if(B3_put==1){display_6_8_number(15,4,3);}
//		else if(B3_put==0){display_6_8_number(15,4,0);}		
//		if(B4_put==1){display_6_8_number(22,4,4);}
//		else if(B4_put==0){display_6_8_number(22,4,0);}		
  }
}





/***************************************
函数名:	void duty_200hz(void)
说明: 200hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void maple_duty_200hz(void)
{
////============================================此处编写5ms中断代码=========================================////

		speed_left=get_right_motor_speed();//获取编码器值
	  speed_right=get_left_motor_speed();//获取编码器值
		button_scan();//按键
	  if(flag==0)steering_ring();		//标志位为0，判断为转向环					
		else if(flag==1)Track_ring();		//标志位为1，判断为寻迹环
//    DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_27);

	
	
////============================================此处编写5ms中断代码=========================================////
}

/***************************************
函数名:	void duty_1000hz(void)
说明: 1000hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_1000hz(void)
{
////============================================此处编写1ms中断代码=========================================////
	
	
	
	

	
	
////============================================此处编写1ms中断代码=========================================////
}


/***************************************
函数名:	void duty_100hz(void)
说明: 100hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_100hz(void)
{
////============================================此处编写10ms中断代码=========================================////
	
	
//  DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_27);
//	PWM_Output(0,3000,0,0);	

	
////============================================此处编写10ms中断代码=========================================////
}


/***************************************
函数名:	void duty_10hz(void)
说明: 10hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_10hz(void)
{
////============================================此处编写100ms中断代码=========================================////
	
	
	
	

	
	
////============================================此处编写100ms中断代码=========================================////
}


void steering_ring(void)//转向�?
{              
	pid_turn=turn_PID_yaw(&yaw_pid,Yaw,L_Target_Position);

	straight_right=(int16_t)pid1(&right_pid,Encoder1,L_Target_Speed+pid_turn);//
	straight_left=(int16_t)pid1(&left_pid,Encoder2,L_Target_Speed-pid_turn);//

	if(straight_right>7200)straight_right=7200;
	if(straight_left>7200)straight_left=7200;	
	if(straight_right<-7200)straight_right=-7200;
	if(straight_left<-7200)straight_left=-7200;	

	Load(straight_left,straight_right);
}
void Track_ring(void)//寻迹环
{
	track_turn=track_pid(&track_pid_assignment,track_sum);
	
  
  track_left=(int16_t)pid1(&left_pid,Encoder2,30-track_turn);//
  track_right=(int16_t)pid1(&right_pid,Encoder1,30+track_turn);//
  
	
	if(track_left>7200)track_left=7200;
	if(track_right>7200)track_right=7200;	
	if(track_right<-7200)track_right=-7200;
	if(track_left<-7200)track_left=-7200;	
	
	Load(track_left,track_right);
}
void Load(int16_t left_pwm1, int16_t right_pwm1)
{
    // 处理左轮方向
    int16_t left_forward = (left_pwm1 > 0) ? left_pwm1 : 0;
    int16_t left_backward = (left_pwm1 < 0) ? -left_pwm1 : 0;

    // 处理右轮方向
    int16_t right_forward = (right_pwm1 > 0) ? right_pwm1 : 0;
    int16_t right_backward = (right_pwm1 < 0) ? -right_pwm1 : 0;

    // 调用PWM输出函数，参数顺序：
    // 左前进，左后退，右后退，右前进
//    PWM_Output(left_forward, 
//              left_backward,
//              right_backward,
//              right_forward);
	
    PWM_Output(right_backward, 
              right_forward,
              left_forward,
              left_backward);
	
}
void one(void)
{
	L_Target_Speed=30;
	L_Target_Position=0;
	while(1)
	{
		Track_follow();
		L_Target_Speed=30;
		L_Target_Position=0;
		if(All_Sensor1){L_Target_Speed=0;break;}
	}

}

void two(void)
{
	uint8_t case2;
	while(1)
	{
		Track_follow();
		switch(case2)
		{
			case 0:
			{
				L_Target_Speed=30;
				L_Target_Position=0;
				flag=0;
				case2=1;				
			}break;
			case 1:
			{
				if(One_Sensor1){flag=1;case2=2;}
			}break;
			case 2:
			{
				if(All_Sensor1){L_Target_Speed=0;L_Target_Position=177;flag=0;case2=3;}
			}break;
			case 3:
			{
				if(yaw==177){L_Target_Speed=30;L_Target_Position=177;case2=4;}
			}break;
			case 4:
			{
				if(One_Sensor1){flag=1;case2=5;}
			}break;
			case 5:
			{
				if(All_Sensor1){L_Target_Speed=0;L_Target_Position=0;flag=3;case2=6;}
			}break;
			case 6:
			{
				Load(0,0);
				case2=7;
			}
		}
		if(case2==7){break;}
	}
	
}

void three(void)
{
  while(1)
  {
    static uint8_t three_prosses=0;     //分成四部分
//    uint8_t black_cnt=0;        // 存储进入圆环次数  
    display_6_8_number(1 ,1,Yaw);
//    OLED_ShowSignedNum(3,5,three_prosses,3);
//    OLED_ShowSignedNum(3,5,L_Target_Position,3);
//    OLED_ShowSignedNum(4,5,pid_turn,3);
//		OLED_ShowSignedNum(1,5,One_Sensor1,3);
//		OLED_ShowSignedNum(3,5,L_Target_Position,3);
//		OLED_ShowSignedNum(4,5,three_prosses,3);
		
//    if(flag!=3){L_Target_Speed = 20;}  
     Track_follow();
    switch(three_prosses)
    {		
      //直行，调整角度，向圆环	
      case 0:
      {
				L_Target_Speed=30;
				L_Target_Position=-38;
				flag=0;
				if(One_Sensor1)
				{					
           three_prosses=1;		
				}
      }break;
       // 识别黑线，先调整角度 
      case 1:
      {       
         L_Target_Speed=0;			
         L_Target_Position=0; 

         if((int16_t)Yaw==0)
				 {
						three_prosses=2;
				 }
      }break;
      //第一次进入圆环，当全为白色，调整角度，直行
      case 2:
      {       
        flag=1;       // 进入寻迹环      
				if(All_Sensor1) //出寻迹环
				{
             three_prosses=3;
				}
      }break;
      case 3:
      {
        L_Target_Speed=30; 
				L_Target_Position=-143;//调整角度
        
        flag=0;
        if(One_Sensor1)
        {          
          flag=1;        //寻迹环
          three_prosses=4;
        }
      }break;
      //识别到全白，停止
      case 4:
      {
				if(All_Sensor1 && Yaw<-130 && Yaw>-179)
				{
					track_sum=-4;
				}
        if(All_Sensor1 && Yaw <30 &&  Yaw>-30)
        {
          // L_Target_Position=0;
          L_Target_Speed=0;
          pid_turn=0;
          flag=0;
          three_prosses=5;
        }
      }break;
      case 5:
      {
				flag=3;
        Load(0,0);
      }
    } 
    
    // if(key_pd[1].keys_read==0){OLED_ShowString(1,1,"stop  ");flag=3;Load(0,0);break; }  
  }
}


//================================师兄的变量================================================//
extern int32_t _encoder_l_count ;
extern int16 left_gpio_dianping;
extern int16 right_gpio_dianping;
extern int16 left_cnt;
extern int16 right_cnt;

extern float Get_Encoder_countA1;


///motor
float speed_left_aul=0;
float speed_right_aul=0;
float speed_left_en=0;
float speed_right_en=0;
float speed_tal_L=12;
float speed_tal_R=12;
float speed_left_out=0;
float speed_right_out=0;
PID leftspeed;
PID rightspeed;

//直线//角度环

float speed_angle_val_renwu1=0;//实时固定角度
int get_speed_angle_val=0;

float speed_angle_out_renwu1=0;

PID angle_Tal_renwu1;

//@@@@@pid

PID rightspeed_Calc;

PID leftspeed_Calc;


//灰度
float err_huidu_out;
float err_turn_out=0;
extern float gray_status[2];

int huidu1,huidu2,huidu3,huidu4,huidu5,huidu6,huidu7,huidu8,huidu9,huidu10,huidu11,huidu12;

float err_aul_huidu=0;
PID erraul_huidu;

//angle

PID angle_Z;
PID gudingangle_Z;
PID leftspeed_angle;
PID rightspeed_angle;

float  gudingangle_Z_out=0;

float angle_Tal=0;
float speed_angletal_L=12;
float speed_angletal_R=12;
float speed_angle_out=0;//角度误差pwm

//imu
float roll_mpu=0;
float pith_mpu=0;
float anglez_mpu=0;
float anglez_mpu2=0;

float anglez_mpu3=0;
float anglez_mpu3_buchang=0;

float temp_anglez_time1=1;

//angle_解算

uint8_t ID;//??????ID????
int16_t AX, AY, AZ, GX, GY, GZ;//?????????????
//////////////////////////mpu??
double Gyro_z=0, Gyro_x=0, Gyro_y=0;
float Acc_x,Acc_y;
double fil_Acc_x,fil_Acc_y,fil_Gyro_z,fil_Gyro_x,fil_Gyro_y;
double Angle_Z=0, Angle_Y=0, Angle_X=0;
float coe_Gyro_z=0.2;
float ICM20602_FIFO_Z[11], ICM20602_FIFO_Y[11], ICM20602_FIFO_X[11];
float imu660ra_acc[11];
int moto_flag=0;
int gyro_i=0;
