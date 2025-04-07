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

short yaw;
short angle;

float speed_right,speed_left;

extern float Pitch,Roll,Yaw;

int main(void)
{
	usart_irq_config();     //串口中断配置
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
	
  while(1)
  {
		
		display_6_8_number(1 ,1,Yaw);
//		display_6_8_number(1 ,2,Roll);
//		display_6_8_number(1 ,3,Pitch);
//		display_6_8_number(1 ,5,speed_right);
//		display_6_8_number(1 ,6,speed_left);
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

		speed_right=get_right_motor_speed();//获取编码器值
		speed_left=get_left_motor_speed();//获取编码器值


	
	
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
	
	

		PWM_Output(3000,0,0,3000);	

	
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
