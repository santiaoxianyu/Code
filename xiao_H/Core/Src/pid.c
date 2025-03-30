#include "pid.h"

/* 
    Encoder1对应板子上“电机1”，right_pid；
    Encoder2对应板子上“电机2”，left_pid；
*/

float  add;
static	int8_t err_last;	
extern unsigned int  rawGyroZ;
// extern travk_read[8];//灰度获取值变量
// extern track_assignment[8];
// extern track_sum;
//转向环变量
//int8_t flag;
// int16_t straight_right;
// int16_t straight_left;
// int16_t pid_turn;
extern float yawl;
//YawProcessor_t processor;//yaw结构定义
//编码器
extern int Encoder1,Encoder2;
extern int16_t L_Target_Position, L_Target_Speed;
extern int8_t track_sum;
//int16_t left,right,left_last,right_last;
//寻迹环变量
//int16_t track_turn,track_left,track_right;

// //板子上的电机2的PID控制器
// PID_Controller left_pid = {
// //	.p = 700.0f,
// //	.i = 0.5f,
// //  .d = 0.0f	
// //上一份代码的p.i
// 	// .p = 380.0f,
// 	// .i = 15.4f,
// 	.p=0.0f,
// 	.i=0.0f
// }; 
// //板子上的电机1的PID控制器
// PID_Controller right_pid = {
// //	.p = 0.0f,
// //	.i = 0.0f,
// //上一份代码的p.i
// 	// .p = 380.0f,	
// 	// .i = 14.0f,
// 	.p=0.0f,
// 	.i=0.0f
// }; 

// PID_Controller turn_pid ={
// //上一份p.d	
//   	// .p=50.0f,
// 	// .d=0.0f
// 	.p=0.0f,
// 	.d=0.0f
// };

// PID_Controller yaw_pid={//直走pid控制器
// //上一份p.d	
// 	.p=0.0f,
// 	.d=0.0f
// 	// .p=0.68f,
// 	// .d=4.0f
	
// //	.d=0.035f
	
// //	.p=940.0f,
// //	.d=0.93f
// //	
// //	.p=0.625f
// //  .p=1.1f,
// //	.d=0.0015f
// //	.d=0.003f
// };

// PID_Controller angle_yaw={//打角pid控制器
// //上一份p.d	
// 	// .p=0.05f,
// 	// .d=0.0f
// 	.p=0.0f,
// 	.d=0.0f
// };


// PID_Controller track_pid_assignment={//寻迹pid控制器
// //上一份p.d	
// 	// .p=2.0f,
// 	// .d=0.0f
// 	.p=0.0f,
// 	.d=0.0f

// };

void pwm_control(float *pwm1)
{
    if(*pwm1>7200)
        *pwm1=7200;
    if(*pwm1<-7200)
        *pwm1=-7200;
}

float pid1(PID_Controller *pid,int16_t speed1,float tar1)
{
//    speed1=myabs(speed1);
    pid->Err=tar1-speed1;
	
    add=pid->p*(pid->Err - pid->last_err)+pid->i*(pid->Err);
//	+d*(Err+next_err-2*last_err) next_err=last_err;
	
    pid->pwm+=add;
	
    pwm_control(&pid->pwm);
   
    pid->last_err=pid->Err;
	
    return pid->pwm;
}

float track_pid(PID_Controller *pid,int8_t  track_err)//寻迹pid控制器-last_err
{
	float pwm;
	int8_t derivative;
	
	derivative=track_err-pid->last_err;
	
	pwm=pid->p*track_err+pid->d*derivative;
	
	pid->last_err=track_err;
		
	return pwm;
}

float turn_PID_yaw(PID_Controller *pid,float yaw,float aim)
{
	float pwm;
	float deviation;
	float yaw_err;
	float kp_out;
	
	yaw_err=aim-yaw;
	
	pid->sum+=yaw_err;
	
	if(pid->sum>100)pid->sum=100;
	if(pid->sum<-100)pid->sum=-100;
	
	deviation=yaw_err - pid->last_err;
	
//	kp_out=	pid->p*yaw_err;
		
	pwm=pid->p*yaw_err+pid->d*deviation+pid->i*pid->sum;//-0.12*rawGyroZ
			
	pid->last_err=yaw_err;
	
	return pwm;	
}

// void steering_ring()//转向环
// {              
// 	Encoder1=Read_Speed(&htim2);
// 	Encoder2=Read_Speed(&htim3);    
// //	if(processor.yaw>-100)
// //	{
// 	pid_turn=turn_PID_yaw(&yaw_pid,yawl,L_Target_Position);
// //	OLED_ShowSignedNum(3,1,yawl,3);
// 		// pid_turn=turn_PID_yaw(&yaw_pid,processor.yaw ,L_Target_Position);	
// //	}
// //	if(processor.yaw<-100)
// //	{
// //	  pid_turn=turn_PID_yaw(&yaw_pid,processor.yaw ,-180 );	
// //	}
// //	if(pid_turn>4)pid_turn=3;
// //	if(pid_turn<-4)pid_turn=-3;+pid_turn)
// //	else pid_turn=turn_PID_yaw(&yaw_pid,processor.yaw , );

// 	// straight_right=(int16_t)pid1(&right_pid,right,L_Target_Speed+pid_turn);
// 	// straight_left=(int16_t)pid1(&left_pid,left,L_Target_Speed-pid_turn);

// 	straight_right=(int16_t)pid1(&right_pid,Encoder1,L_Target_Speed+pid_turn);
// 	straight_left=(int16_t)pid1(&left_pid,Encoder2,L_Target_Speed-pid_turn);

// //	straight_right=pid1(&right_pid,right,pid_turn);
// //	straight_left=pid1(&left_pid,left,-pid_turn);
// 	if(straight_right>7200)straight_right=7200;
// 	if(straight_left>7200)straight_left=7200;	
// 	if(straight_right<-7200)straight_right=-7200;
// 	if(straight_left<-7200)straight_left=-7200;	

// 	Load(straight_left,straight_right);
// }


// void Track_ring()//寻迹环
// {
// 	track_turn=track_pid(&track_pid_assignment,track_sum);
// 	track_left=pid1(&left_pid,Encoder1,25-track_turn);
// 	track_right=pid1(&right_pid,Encoder2,25+track_turn);
// 	if(track_left>7200)track_left=7200;
// 	if(track_right>7200)track_right=7200;	
// 	Load(track_left,track_right);
// }

