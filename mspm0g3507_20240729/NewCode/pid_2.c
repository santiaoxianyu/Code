#include "pid_2.h"

float  add;

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
float track_pid(PID_Controller *pid,int8_t  track_err)//Ñ°¼£pid¿ØÖÆÆ÷-last_err
{
	static int16_t track_sum;
	float pwm;
	int8_t derivative;
	
	track_sum+=track_err;
	
	derivative=track_err-pid->last_err;
	
	pwm=pid->p*track_err+pid->d*derivative+pid->i*track_sum;
	
	pid->last_err=track_err;
		
	return pwm;
}
float turn_PID_yaw(PID_Controller *pid,float yaw,float aim)
{
	float pwm;
	float deviation;
	float yaw_err;
	float kp_out;
	
	yaw_err=aim-yaw;//-143-150=-293+180=-110
	if(yaw_err<-200)
	{
		yaw_err=-(yaw_err+250);
	}
//	else if(yaw_err>200)
//	{
//		yaw_err=-(yaw_err-250);
//	}
	pid->sum+=yaw_err;
	
	if(pid->sum>100)pid->sum=100;
	if(pid->sum<-100)pid->sum=-100;
	
	deviation=yaw_err - pid->last_err;
	
//	kp_out=	pid->p*yaw_err;
		
	pwm=pid->p*yaw_err+pid->d*deviation+pid->i*pid->sum;//-0.12*rawGyroZ
			
	pid->last_err=yaw_err;
	
	return pwm;	
}
