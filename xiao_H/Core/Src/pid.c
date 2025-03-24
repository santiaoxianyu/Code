#include "pid.h"

extern float gyro_z;

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
		
	pwm=pid->p*yaw_err+pid->d*deviation+pid->i*pid->sum-0.12*gyro_z;
			
	pid->last_err=yaw_err;
	
	return pwm;	
}


