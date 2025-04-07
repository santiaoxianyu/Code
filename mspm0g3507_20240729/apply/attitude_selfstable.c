#include "headfile.h"
#include "attitude_selfstable.h"

controller roll_angle_ctrl,pitch_angle_ctrl,yaw_angle_ctrl;//�ǶȻ�
controller roll_gyro_ctrl,pitch_gyro_ctrl,yaw_gyro_ctrl;//���ٶȻ�

void attitude_ctrl_init(void)
{
	pid_control_init(&roll_angle_ctrl,//����ʼ���������ṹ��
										5.0,//��������
										0,  //���ֲ���
										0,  //΢�ֲ���
										30, //ƫ���޷�ֵ
										0,  //�����޷�ֵ
										500,//����������޷�ֵ
										1,  //ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��
	pid_control_init(&pitch_angle_ctrl,//����ʼ���������ṹ��
										5.0,//��������
										0,  //���ֲ���
										0,  //΢�ֲ���
										30, //ƫ���޷�ֵ
										0,  //�����޷�ֵ
										500,//����������޷�ֵ
										1,  //ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��
	pid_control_init(&yaw_angle_ctrl,//����ʼ���������ṹ��
										5.0,//��������
										0,  //���ֲ���
										0,  //΢�ֲ���
										30, //ƫ���޷�ֵ
										0,  //�����޷�ֵ
										500,//����������޷�ֵ
										1,  //ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��

	pid_control_init(&roll_gyro_ctrl,//����ʼ���������ṹ��
										1.0,//��������
										3.0,  //���ֲ���
										4.5,  //΢�ֲ���
										500, //ƫ���޷�ֵ
										300,  //�����޷�ֵ
										1000,//����������޷�ֵ
										1,  //ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��
										
	pid_control_init(&pitch_gyro_ctrl,//����ʼ���������ṹ��
										1.0,//��������
										3.0,  //���ֲ���
										4.5,  //΢�ֲ���
										500, //ƫ���޷�ֵ
										300,  //�����޷�ֵ
										1000,//����������޷�ֵ
										1,  //ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��

	pid_control_init(&yaw_gyro_ctrl,//����ʼ���������ṹ��
										2.0,//��������
										1.0,  //���ֲ���
										2.0,  //΢�ֲ���
										200, //ƫ���޷�ֵ
										200,  //�����޷�ֵ
										800,//����������޷�ֵ
										1,  //ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��
										
	set_cutoff_frequency(200,20,&roll_angle_ctrl.lpf_params);
	set_cutoff_frequency(200,20,&pitch_angle_ctrl.lpf_params);
	set_cutoff_frequency(200,20,&yaw_angle_ctrl.lpf_params);										
	set_cutoff_frequency(200,20,&roll_gyro_ctrl.lpf_params);
	set_cutoff_frequency(200,20,&pitch_gyro_ctrl.lpf_params);
	set_cutoff_frequency(200,20,&yaw_gyro_ctrl.lpf_params);
	
}



void attitude_control(void)//��̬���ơ����Ƕ�+���ٶ�
{			
	//��̬�Ƕ�����
	roll_angle_ctrl.expect =RC_Pilot.rc_rpyt[RC_ROLL];
	pitch_angle_ctrl.expect=RC_Pilot.rc_rpyt[RC_PITCH];	
  //��̬�Ƕȷ���
  roll_angle_ctrl.feedback =smartcar_imu.rpy_deg[_ROL];
	pitch_angle_ctrl.feedback=smartcar_imu.rpy_deg[_PIT];	
	//����PID������
  pid_control_dt_run(&roll_angle_ctrl,0.005f);  
	pid_control_dt_run(&pitch_angle_ctrl,0.005f);
	
	//���������������̬�ڻ����ٶȿ���������PID������
	/***************�ڻ����ٶ�����****************/
	roll_gyro_ctrl.expect=roll_angle_ctrl.output;
	pitch_gyro_ctrl.expect=pitch_angle_ctrl.output;
	/***************�ڻ����ٶȷ���****************/
	roll_gyro_ctrl.feedback=smartcar_imu.rpy_gyro_dps[_ROL];
	pitch_gyro_ctrl.feedback=smartcar_imu.rpy_gyro_dps[_PIT];
	/***************�ڻ����ٶȿ��ƣ�΢�ֲ�����̬����****************/
	pid_control_dt_run(&roll_gyro_ctrl,0.005f);
	pid_control_dt_run(&pitch_gyro_ctrl,0.005f);
	
  //ƫ���ǿ���
	if(RC_Pilot.rc_rpyt[RC_YAW]==0)//ƫ����������λ
	{
		if(yaw_angle_ctrl.expect==0)//�����л���
		{
			yaw_angle_ctrl.expect=smartcar_imu.rpy_deg[_YAW];
		}
		yaw_angle_ctrl.feedback=smartcar_imu.rpy_deg[_YAW];//ƫ���Ƿ���
		pid_control_dt_yaw_run(&yaw_angle_ctrl,0.005f);//ƫ���Ƕȿ���
		yaw_gyro_ctrl.expect=yaw_angle_ctrl.output;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
	}
	else//����ƫ������˺�ֻ�����ڻ����ٶȿ���
	{
		yaw_angle_ctrl.expect=0;//ƫ����������0,�����нǶȿ���
		yaw_gyro_ctrl.expect=RC_Pilot.rc_rpyt[RC_YAW];//ƫ�����ٶȻ�������ֱ����Դ��ң���������
	}	
  yaw_gyro_ctrl.feedback=smartcar_imu.rpy_gyro_dps[_YAW];
  pid_control_dt_run(&yaw_gyro_ctrl,0.005f);
}


void takeoff_ctrl_reset(void)
{
  pid_integrate_reset(&roll_gyro_ctrl);//���ǰ���λ���
  pid_integrate_reset(&pitch_gyro_ctrl);
  pid_integrate_reset(&yaw_gyro_ctrl);
  pid_integrate_reset(&roll_angle_ctrl);
  pid_integrate_reset(&pitch_angle_ctrl);
  pid_integrate_reset(&yaw_angle_ctrl);
}

controller_output maplepilot={
	.unlock_flag=LOCK,
};


bool motor_idel_enable=false;
uint16_t motor_idel_cnt=0;
uint16_t motor_idel_value=0;
float motor_idel_period=5;//20
float motor_idel_time=0,motor_idel_last_time=0;
void ncontroller_output(void)
{
	if(RC_Pilot.lock_state==UNLOCK)//����״̬
	{
		//�����������Դ
		maplepilot.roll_control_output =roll_gyro_ctrl.output;
		maplepilot.pitch_control_output=pitch_gyro_ctrl.output;
		maplepilot.yaw_control_output  =yaw_gyro_ctrl.output;	
	  maplepilot.throttle_control_output=RC_Pilot.thr;
			
		if(motor_idel_enable==true)//������������
		{
			motor_idel_time=millis();
			if(motor_idel_time-motor_idel_last_time>motor_idel_period)
			{
			  motor_idel_last_time=motor_idel_time;
				motor_idel_cnt++;
			}	  
			motor_idel_value=THR_MIN_OUTPUT+motor_idel_cnt;	
			maplepilot.motor_output[MOTOR1]=motor_idel_value;
			maplepilot.motor_output[MOTOR2]=motor_idel_value;
			maplepilot.motor_output[MOTOR3]=motor_idel_value;
			maplepilot.motor_output[MOTOR4]=motor_idel_value;	
			
		  if(motor_idel_cnt>=(THR_IDEL_OUTPUT-THR_MIN_OUTPUT))
			{
			  motor_idel_enable=false;
				motor_idel_cnt=0;
				motor_idel_last_time=motor_idel_time=0;
			}
		}
		else
		{
			if(maplepilot.throttle_control_output>=THR_CONTROL_WORK)
			{
					maplepilot.motor_output[MOTOR1]=maplepilot.throttle_control_output-maplepilot.roll_control_output+maplepilot.pitch_control_output-maplepilot.yaw_control_output;
					maplepilot.motor_output[MOTOR2]=maplepilot.throttle_control_output+maplepilot.roll_control_output-maplepilot.pitch_control_output-maplepilot.yaw_control_output;
					maplepilot.motor_output[MOTOR3]=maplepilot.throttle_control_output+maplepilot.roll_control_output+maplepilot.pitch_control_output+maplepilot.yaw_control_output;
					maplepilot.motor_output[MOTOR4]=maplepilot.throttle_control_output-maplepilot.roll_control_output-maplepilot.pitch_control_output+maplepilot.yaw_control_output;
			}
			else
			{
					maplepilot.motor_output[MOTOR1]=maplepilot.throttle_control_output;
					maplepilot.motor_output[MOTOR2]=maplepilot.throttle_control_output;
					maplepilot.motor_output[MOTOR3]=maplepilot.throttle_control_output;
					maplepilot.motor_output[MOTOR4]=maplepilot.throttle_control_output;
					takeoff_ctrl_reset();//�����
			}			
		}
		
		if(RC_Pilot.auto_relock_flag==1//������ʱ�̣��Զ�������־Ϊ����
			&&RC_Pilot.rc_return_flag==1)//ƫ������ʱ�����
		{
		  takeoff_ctrl_reset();//�����
		}
		//������������С���ֵΪ����ֵ
		for(uint16_t i=0;i<4;i++)
		{
			maplepilot.motor_output[i]=constrain_int16(maplepilot.motor_output[i],THR_IDEL_OUTPUT,THR_MAX_OUTPUT);
		}
	}
	else
	{
	  maplepilot.roll_control_output =0;
		maplepilot.pitch_control_output=0;
		maplepilot.yaw_control_output  =0;	
		maplepilot.throttle_control_output=THR_MIN_OUTPUT;
		
		maplepilot.motor_output[MOTOR1]=maplepilot.throttle_control_output;
		maplepilot.motor_output[MOTOR2]=maplepilot.throttle_control_output;
		maplepilot.motor_output[MOTOR3]=maplepilot.throttle_control_output;
		maplepilot.motor_output[MOTOR4]=maplepilot.throttle_control_output;
		
		motor_idel_enable=true;
		motor_idel_cnt=0;
		motor_idel_last_time=motor_idel_time=0;	
		takeoff_ctrl_reset();//�����
	}
	//PWM���
	steer_servo_pwm_m1p0(maplepilot.motor_output[MOTOR1]);
	steer_servo_pwm_m1p1(maplepilot.motor_output[MOTOR2]);
	steer_servo_pwm_m1p2(maplepilot.motor_output[MOTOR3]);
	steer_servo_pwm_m1p3(maplepilot.motor_output[MOTOR4]);
}

