#include "headfile.h"
#include "pid.h"



extern float anglez_mpu3;
/***************************************************
������: void pid_control_init(controller *ctrl,
															float kp,
															float ki,
															float kd,
															float err_max,
															float integral_max,
															float output_max,
															uint8_t err_limit_flag,
															uint8_t integral_separate_flag,
															float integral_separate_limit,
															uint8_t dis_error_gap_cnt)
˵��:	pid��������ʼ��
���:	controller *ctrl-����ʼ���������ṹ��
			float kp-��������
			float ki-���ֲ���
			float kd-΢�ֲ���
			float err_max-ƫ���޷�ֵ
			float integral_max-�����޷�ֵ
			float output_max-����޷�ֵ
			uint8_t err_limit_flag-ƫ���޷���־λ
			uint8_t integral_separate_flag-���ַ����־λ
			float integral_separate_limit-���ַ����޷���־λ
			uint8_t dis_error_gap_cnt-����΢��ʱ�ļ������
����:	��
��ע:	��
****************************************************/
void pid_control_init(controller *ctrl,
											float kp,
											float ki,
											float kd,
											float err_max,
											float integral_max,
											float output_max,
											uint8_t err_limit_flag,
											uint8_t integral_separate_flag,
											float integral_separate_limit,
											uint8_t dis_error_gap_cnt)
{
	ctrl->kp=kp;
	ctrl->ki=ki;
	ctrl->kd=kd;
	ctrl->error_limit_max=err_max;
	ctrl->integral_limit_max=integral_max;
	ctrl->output_limit_max=output_max;
	ctrl->error_limit_flag=err_limit_flag;
	ctrl->integral_separate_flag =integral_separate_flag;
	ctrl->integral_separate_limit=integral_separate_limit;
	
	ctrl->dis_error_gap_cnt=dis_error_gap_cnt;
	
	ctrl->init_flag=1;
}

void  pid_integrate_reset(controller *ctrl)  {ctrl->integral=0.0f;}





///////////////////////////pid/////////////////////////////////--??si




float abs_limit(float value, float ABS_MAX)   //????,??????
{
    if(value > ABS_MAX)
        value = ABS_MAX;

    if(value< -ABS_MAX)
        value = -ABS_MAX;
    return value;
}

float guding_angle( PID *pid,float Target_val,float Actual_val)
	
{
	//�Ƕȿ���
	   float pwm_p;
    float pwm_i;
    pid->Integralmax=500;
	//steerangle_ctrl.expect=steer_angle_expect;
	Actual_val=anglez_mpu3;
	//steerangle_ctrl.feedback=smartcar_imu.rpy_deg[_YAW];
	pid->Error=Target_val-anglez_mpu3;
	pid->SumError += pid->Error; 
		/***********************ƫ����ƫ���+-180����*****************************/
		if(pid->Error<-180) pid->Error=pid->Error+360;
	  if(pid->Error>	180) pid->Error=pid->Error-360;
	  pid->Error=constrain_float(pid->Error,-50,50);
	  pwm_p = pid->Kp* pid->Error;
	 pwm_i = abs_limit( pid->Ki* pid->SumError, pid->Integralmax );
	pid->output =   pwm_p + pwm_i +pid->Kd* pid->DError ;
	
	pid->output=constrain_float(pid->output,-500,500);
	
	 if(pid->output > pid->outputmax )  pid->output = pid->outputmax;
	 if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
   //if(pid->output <=10 )  pid->output = 10;
    return pid->output; 
	//steerangle_ctrl.error=steerangle_ctrl.expect-steerangle_ctrl.feedback;
	/***********************ƫ����ƫ���+-180����*****************************/
	//if(steerangle_ctrl.error<-180) steerangle_ctrl.error=steerangle_ctrl.error+360;
	//if(steerangle_ctrl.error>	180) steerangle_ctrl.error=steerangle_ctrl.error-360;				
	//steerangle_ctrl.error=constrain_float(steerangle_ctrl.error,-50,50);
	//steer_angle_output=steerangle_ctrl.kp*steerangle_ctrl.error;
	//steer_angle_output=constrain_float(steer_angle_output,-500,500);
	//��¼ƫ��ֵ,�����ж�ת��ִ��״̬
	//steer_angle_error=steerangle_ctrl.error;
	
}

float PID_Position_Calc(PID *pid, float Target_val, float Actual_val)  //???PID
{
    float pwm_p;
    float pwm_i;
    pid->Integralmax=500;

    pid->Error = Target_val - Actual_val;      //?pid P?????????? ????=???-???
    pid->SumError += pid->Error;                 //?pid I?????????? ??????????,????
    pid->DError = pid->Error - pid->LastError;   //?pid D????? ???-????
    pwm_p = pid->Kp* pid->Error;
    pwm_i = abs_limit( pid->Ki* pid->SumError, pid->Integralmax );

    pid->output =   pwm_p + pwm_i +pid->Kd* pid->DError ;
   pid->LastError = pid->Error; 
    if(pid->output > pid->outputmax )  pid->output = pid->outputmax;
	
	
    if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
    if(pid->output <=10 )  pid->output = 10;
    return pid->output ;  
}
float PID_Incremental_Calc(PID *pid, float Target_val, float Actual_val)
{

    float pwm_p;
    float pwm_i;

    pid->Error = Target_val- Actual_val;



    pwm_p= pid->Kp* ( pid->Error - pid->LastError );
    pwm_i=pid->Ki* pid->Error;
if(pwm_i>=100)pwm_i=100;
    pid->output  +=  pwm_p+
                     pwm_i +
                     pid->Kd* ( pid->Error +  pid->PrevError - 2*pid->LastError);

    
    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;

    if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
    //if(pid->output <=100)  pid->output =100 ;
    if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
   if(pid->output <=10)  pid->output =10 ;
    return pid->output ;   //???pwm?
}





void PID_Init(PID *pid, float Kp , float Ki , float Kd , float Limit_value)  
{
    pid->Kp= Kp;
    pid->Ki= Ki;
    pid->Kd= Kd;

    pid->PrevError =pid->LastError = pid->Error =pid->SumError= pid->output =  0;
    pid->Integralmax = pid->outputmax  = Limit_value;
}













/***************************************************
������: float pid_control_run(controller *ctrl)
˵��:	pid����������
���:	controller *ctrl-�������ṹ��
����:	��
��ע:	��
****************************************************/






















float pid_control_run(controller *ctrl)
{
  /*******ƫ�����*********************/
	for(uint16_t i=19;i>0;i--)
	{
		ctrl->error_backup[i]=ctrl->error_backup[i-1];
	}
	ctrl->error_backup[0]=ctrl->error;
	
  ctrl->last_error=ctrl->error;//�����ϴ�ƫ��
  ctrl->error=ctrl->expect-ctrl->feedback;//������ȥ�����õ�ƫ��
	ctrl->dis_error=ctrl->error-ctrl->error_backup[ctrl->dis_error_gap_cnt-1];//΢��		ctrl->dis_error=ctrl->error-ctrl->last_error;//ԭʼ΢��
	
  if(ctrl->error_limit_flag==1)//ƫ���޷��ȱ�־λ
  {
    if(ctrl->error>= ctrl->error_limit_max)  ctrl->error= ctrl->error_limit_max;
    if(ctrl->error<=-ctrl->error_limit_max)  ctrl->error=-ctrl->error_limit_max;
  }
  /*******���ּ���*********************/
  if(ctrl->integral_separate_flag==1)//���ַ����־λ
  {
		//ֻ��ƫ��Ƚ�С��ʱ��������ֿ���
    if(ABS(ctrl->error)<=ctrl->integral_separate_limit)	 ctrl->integral+=ctrl->ki*ctrl->error;
  }
  else
  {
    ctrl->integral+=ctrl->ki*ctrl->error;
  }
	/*******�����޷�*********************/
	if(ctrl->integral>=ctrl->integral_limit_max)   ctrl->integral=ctrl->integral_limit_max;
	if(ctrl->integral<=-ctrl->integral_limit_max)  ctrl->integral=-ctrl->integral_limit_max;
	
	/*******���������*********************/
  ctrl->last_output=ctrl->output;//���ֵ����
  ctrl->output=ctrl->kp*ctrl->error//����
							+ctrl->integral//����
							+ctrl->kd*ctrl->dis_error;//΢��
	
	/*******������޷�*********************/
  if(ctrl->output>= ctrl->output_limit_max)  ctrl->output= ctrl->output_limit_max;
  if(ctrl->output<=-ctrl->output_limit_max)  ctrl->output=-ctrl->output_limit_max;
  /*******���������*********************/
  return ctrl->output;
}


/////////////////////////////////
float pid_control_dt_run(controller *ctrl,float dt)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
	if(_dt>1.05f*dt||_dt<0.95f*dt||isnan(_dt)!=0)   _dt=dt;
	if(_dt<0.0001f) return 0;

  /*******ƫ�����*********************/
	for(uint16_t i=19;i>0;i--)
	{
		ctrl->error_backup[i]=ctrl->error_backup[i-1];
	}
	ctrl->error_backup[0]=ctrl->error;
	
  ctrl->last_error=ctrl->error;//�����ϴ�ƫ��
  ctrl->error=ctrl->expect-ctrl->feedback;//������ȥ�����õ�ƫ��
	ctrl->dis_error=ctrl->error-ctrl->error_backup[ctrl->dis_error_gap_cnt-1];//΢��		ctrl->dis_error=ctrl->error-ctrl->last_error;//ԭʼ΢��

  if(ctrl->error_limit_flag==1)//ƫ���޷��ȱ�־λ
  {
    if(ctrl->error>= ctrl->error_limit_max)  ctrl->error= ctrl->error_limit_max;
    if(ctrl->error<=-ctrl->error_limit_max)  ctrl->error=-ctrl->error_limit_max;
  }
  /*******���ּ���*********************/
  if(ctrl->integral_separate_flag==1)//���ַ����־λ
  {
		//ֻ��ƫ��Ƚ�С��ʱ��������ֿ���
    if(ABS(ctrl->error)<=ctrl->integral_separate_limit)	 ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
  else
  {
    ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
	/*******�����޷�*********************/
	if(ctrl->integral>=ctrl->integral_limit_max)   ctrl->integral=ctrl->integral_limit_max;
	if(ctrl->integral<=-ctrl->integral_limit_max)  ctrl->integral=-ctrl->integral_limit_max;
	
	ctrl->dis_error_lpf=LPButterworth(ctrl->dis_error,&ctrl->lpf_buffer,&ctrl->lpf_params);
		
	/*******���������*********************/
  ctrl->last_output=ctrl->output;//���ֵ����
  ctrl->output=ctrl->kp*ctrl->error//����
							+ctrl->integral//����
							+ctrl->kd*ctrl->dis_error_lpf;//΢��
	
	/*******������޷�*********************/
  if(ctrl->output>= ctrl->output_limit_max)  ctrl->output= ctrl->output_limit_max;
  if(ctrl->output<=-ctrl->output_limit_max)  ctrl->output=-ctrl->output_limit_max;
  /*******���������*********************/
  return ctrl->output;
}
//////////////////

float pid_control_dt_yaw_run(controller *ctrl,float dt)
{
  float _dt=0;
  get_systime(&ctrl->_time);
  _dt=ctrl->_time.period/1000.0f;
	if(_dt>1.05f*dt||_dt<0.95f*dt||isnan(_dt)!=0)   _dt=dt;
	if(_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
	for(uint16_t i=19;i>0;i--)
	{
		ctrl->error_backup[i]=ctrl->error_backup[i-1];
	}
	ctrl->error_backup[0]=ctrl->error;
	
  ctrl->last_error=ctrl->error;//�����ϴ�ƫ��
  ctrl->error=ctrl->expect-ctrl->feedback;//������ȥ�����õ�ƫ��
	ctrl->dis_error=ctrl->error-ctrl->error_backup[ctrl->dis_error_gap_cnt-1];//΢��		ctrl->dis_error=ctrl->error-ctrl->last_error;//ԭʼ΢��

  /***********************ƫ����ƫ���+-180����*****************************/
	if(ctrl->error<-180) ctrl->error=ctrl->error+360;
	if(ctrl->error>180)  ctrl->error=ctrl->error-360;

	
  if(ctrl->error_limit_flag==1)//ƫ���޷��ȱ�־λ
  {
    if(ctrl->error>= ctrl->error_limit_max)  ctrl->error= ctrl->error_limit_max;
    if(ctrl->error<=-ctrl->error_limit_max)  ctrl->error=-ctrl->error_limit_max;
  }
  /*******���ּ���*********************/
  if(ctrl->integral_separate_flag==1)//���ַ����־λ
  {
		//ֻ��ƫ��Ƚ�С��ʱ��������ֿ���
    if(ABS(ctrl->error)<=ctrl->integral_separate_limit)	 ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
  else
  {
    ctrl->integral+=ctrl->ki*ctrl->error*_dt;
  }
	/*******�����޷�*********************/
	if(ctrl->integral>=ctrl->integral_limit_max)   ctrl->integral=ctrl->integral_limit_max;
	if(ctrl->integral<=-ctrl->integral_limit_max)  ctrl->integral=-ctrl->integral_limit_max;
	
	/*******���������*********************/
  ctrl->last_output=ctrl->output;//���ֵ����
  ctrl->output=ctrl->kp*ctrl->error//����
							+ctrl->integral//����
							+ctrl->kd*ctrl->dis_error;//΢��
	
	/*******������޷�*********************/
  if(ctrl->output>= ctrl->output_limit_max)  ctrl->output= ctrl->output_limit_max;
  if(ctrl->output<=-ctrl->output_limit_max)  ctrl->output=-ctrl->output_limit_max;
  /*******���������*********************/
  return ctrl->output;
}

