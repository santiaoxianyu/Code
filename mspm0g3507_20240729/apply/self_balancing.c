#include "headfile.h"
#include "self_balancing.h"
#include "user.h"


#define balance_deadzone 75				 //ƽ�⻻���������  50
#define drive_brake_angle_max   30 //���ɲ�������Ƕ�  30

float balance_angle_expect=balance_angle_default;//�����Ƕ����
float	balance_angle_output=0,balance_gyro_output=0;//���������
float balance_speed_scale=balance_speed_scale_default;//�ٶȿ��������ת����������������̬�Ƕ�֮���ת��ϵ��
float balance_angle_delta=0;//������������̬�Ƕ�
uint8_t balance_speed_ctrl_enable=1,balance_steer_ctrl_enable=1,balance_ctrl_loop_num=1;//�Լ����ڲ���ʱ,�ٶ�/ת�����ʹ��

//��ƽ��Ƕȿ������ṹ��
controller selfbalance_angle1_ctrl=
{
	.kp=balance_angle_kp1_default,  			//��������
	.ki=balance_angle_ki1_default,  			//���ֲ���
	.kd=balance_angle_kd1_default,  			//΢�ֲ���
	.error_limit_max=75.0f,  		//ƫ���޷�ֵ
	.integral_limit_max=0,			//�����޷�ֵ
	.output_limit_max=500,			//����������޷�ֵ
	.error_limit_flag=1,  			//ƫ���޷���־λ
	.integral_separate_flag=0,  //���ַ����־λ
	.integral_separate_limit=0, //������ֿ���ʱ���޷�ֵ
	.dis_error_gap_cnt=1,				//΢�ּ��ʱ��
};		

/***************************************************
������: void balance_control_single_control(void)
˵��:	���ǶȻ���ƽ�����
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void balance_control_single_control(void)
{
	selfbalance_angle1_ctrl.expect=balance_angle_expect;//����
	selfbalance_angle1_ctrl.feedback=smartcar_imu.rpy_kalman_deg[PIT];	//����		smartcar_imu.rpy_deg[PIT]
	
	selfbalance_angle1_ctrl.error=selfbalance_angle1_ctrl.expect-selfbalance_angle1_ctrl.feedback;
	selfbalance_angle1_ctrl.error=constrain_float(selfbalance_angle1_ctrl.error,-75.0f,75.0f);
	selfbalance_angle1_ctrl.output=selfbalance_angle1_ctrl.kp*selfbalance_angle1_ctrl.error+selfbalance_angle1_ctrl.kd*(-smartcar_imu.rpy_gyro_dps[PIT]);
	
	balance_angle_output=selfbalance_angle1_ctrl.output;
	balance_angle_output=constrain_float(balance_angle_output,-10000,10000);
	
	//��ƽ����ƵĻ�����,�����ٶȿ������
	motion_ctrl_pwm=balance_angle_output-speed_smooth_output*balance_speed_ctrl_enable;  
	//������������
	if(motion_ctrl_pwm>0) motion_ctrl_pwm+=balance_deadzone;
	if(motion_ctrl_pwm<0) motion_ctrl_pwm-=balance_deadzone;
}




controller selfbalance_angle2_ctrl=
{
	.kp=balance_angle_kp2_default,  			//��������
	.ki=balance_angle_ki2_default,  			//���ֲ���
	.kd=balance_angle_kd2_default,  			//΢�ֲ���
	.error_limit_max=20,  			//ƫ���޷�ֵ
	.integral_limit_max=0,			//�����޷�ֵ
	.output_limit_max=500,			//����������޷�ֵ
	.error_limit_flag=1,  			//ƫ���޷���־λ
	.integral_separate_flag=0,  //���ַ����־λ
	.integral_separate_limit=0, //������ֿ���ʱ���޷�ֵ
	.dis_error_gap_cnt=1,				//΢�ּ��ʱ��
};
controller selfbalance_gyro_ctrl;		  //��ƽ����ٶȿ������ṹ��


/***************************************************
������: void balance_control_double_closed_loop(void)
˵��:	�Ƕ�+���ٶȻ���ƽ�����
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void balance_control_double_closed_loop(void)
{	
	//�����ٶȿ��������ֵ������ӳ�����̬�Ƕȱ仯��
	balance_angle_delta=balance_speed_scale*speed_smooth_output;
	balance_angle_delta=constrain_float(balance_angle_delta,-drive_brake_angle_max,drive_brake_angle_max);
  //�Ƕȿ���
	selfbalance_angle2_ctrl.expect=(balance_angle_expect-balance_angle_delta);//����
	selfbalance_angle2_ctrl.feedback=smartcar_imu.rpy_deg[PIT];							 //����
	pid_control_run(&selfbalance_angle2_ctrl);		  													 //����������		
	balance_angle_output=selfbalance_angle2_ctrl.output;
	balance_angle_output=constrain_float(balance_angle_output,-500,500);
	//���ٶȿ���
	selfbalance_gyro_ctrl.expect=balance_angle_output;//����
	selfbalance_gyro_ctrl.feedback=smartcar_imu.rpy_gyro_dps[PIT];//����
	pid_control_run(&selfbalance_gyro_ctrl);		  //����������
	balance_gyro_output=selfbalance_gyro_ctrl.output;
	//��������޷�
	balance_gyro_output=constrain_float(balance_gyro_output,-999,999);
}



//balance_angle_delta=balance_speed_scale*speed_smooth_output*balance_speed_ctrl_enable;
//balance_angle_delta=constrain_float(balance_angle_delta,-15,15);
//balance_angle_delta=0;
//selfbalance_angle1_ctrl.expect=(balance_angle_expect-balance_angle_delta);//����

