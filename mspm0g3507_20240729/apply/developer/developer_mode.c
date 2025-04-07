#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "developer_mode.h"

int16_t sdk_work_mode=0;


#define wheel_space_cm  12.8f//�ּ��  12.8cm



void sdk_duty_run(void)
{
	if(trackless_output.init==0)
	{		
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output=0;
		trackless_output.init=1;
		flight_subtask_reset();//��λsdk������״̬��
	}
	if(smartcar_imu.imu_convergence_flag!=1) return;//��̬����ϵͳ��λ
	
	switch(sdk_work_mode)
	{
		case -10://��ʼ����ģʽ������ȷ������˶�����ʱʹ��
		{
			speed_ctrl_mode=0;  //ֱ�ӿ������ָ��PWM��ֵ�����ڵ��Ե������
			motion_ctrl_pwm=motion_test_pwm_default;//Ĭ������ٷ�֮50ռ�յ�pwm
		}
		break;
		case -9://��ʼ����ģʽ������ȷ�������ֵʱʹ��
		{
			speed_ctrl_mode=0;  //ֱ�ӿ������ָ��PWM��ֵ�����ڵ��Ե������
			motion_ctrl_pwm=0;//Ĭ������ٷ�֮50ռ�յ�pwm
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		}
		break;
		case -2://���ٲ���ģʽ�����ٶ�������Դ��ң��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���
			speed_expect[0]=speed_setup;//��������ٶ�����
			speed_expect[1]=speed_setup;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;
		case -1://���ٲ���ģʽ�����ٶ�������Դ�ڰ����趨
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			speed_expect[0]=speed_setup;//��������ٶ�����
			speed_expect[1]=speed_setup;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;
		case 0://ң�ؿ���
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			trackless_output.yaw_ctrl_mode=ROTATE;//ƫ������ģʽ
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//ƫ��������Դ�ں���˸���		
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);				
		}
		break;		
		case 1://���ڻҶȹܵ�����Ѱ��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			gray_turn_control_200hz(&turn_ctrl_pwm);//���ڻҶȶԹܵ�ת�����
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;		
		case 2://˳ʱ��ת��90��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������			
			flight_subtask_1();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 3://��ʱ��ת��90��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������			
			flight_subtask_2();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 4://˳ʱ��ת��90��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������			
			flight_subtask_3();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 5://��30deg/s�Ľ��ٶ���ʱ��ת��3000ms
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			flight_subtask_4();
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 6://���ڳ���������ǰ���ײС��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			flight_subtask_5();
			steer_control(&turn_ctrl_pwm);	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);
		}
		break;		
		case 7://����ƽ�⳵
		{
			speed_ctrl_mode=2;//�ٶȿ��Ʒ�ʽΪƽ���ٶȿ���
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���
			speed_setup=remote_data_remap(&RC_Data ,RC_PITCH_CHANNEL ,50,50,false);//��ң�˶���λӳ��������ٶ�
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);
			
			trackless_output.yaw_ctrl_mode=ROTATE;//ƫ������ģʽ
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//ƫ��������Դ�ں���˸���
			steer_control(&turn_ctrl_pwm);	//ת�������ǿ���
			balance_control_single_control();			
		}
		break;
		case 8://�������ֲ���ģ�͵��ٶȡ����ٶȿ���,���ڻ��ؼ����ROS�˷����˶�ָ�������λ������ƽ̨
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
	
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//ƫ��������Դ�ں���˸���
			turn_ctrl_pwm=trackless_output.yaw_outer_control_output*DEG2RAD;//�������ٶ�ת���ɻ�����
			
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���
			//�����ٶ�
			speed_expect[0]=speed_setup-turn_ctrl_pwm*wheel_space_cm*0.5f;//��������ٶ�����
			speed_expect[1]=speed_setup+turn_ctrl_pwm*wheel_space_cm*0.5f;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 9://����վ�������ģʽ��ͨ���������µ���վV1.0.6�汾��������
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//�����ٶ�
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);			
		}
		break;		
		case 10://OPENMV�Ӿ�����Ѱ��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 11://˫���+ǰ�ֶ��ת��ң�ؿ���
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+RC_Data.rcdata[RC_YAW_CHANNEL]-1500);	
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup;//��������ٶ�����
			speed_expect[1]=speed_setup;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);	
		}	
		break;
		case 12://˫���+ǰ�ֶ��ת���Ӿ�����Ѱ��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
			//�����ٶ�
			speed_expect[0]=speed_setup;//��������ٶ�����
			speed_expect[1]=speed_setup;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);	
		}
		break;
		case 13://�������
		{
			auto_reverse_stall_park();
		}
		break;
		case 14://�෽ͣ��
		{
			auto_parallel_park();
		}		
		break;
		case 15://2022��7�·�ʡ��С��������ʻϵͳ����,����Ȧ����ѭ��
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			gray_turn_control_200hz(&turn_ctrl_pwm);//���ڻҶȶԹܵ�ת�����
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);		
		}
		break;
		case 16:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 17:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 18:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 19:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 20:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 21:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 22:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		default:
		{
			speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
			trackless_output.yaw_ctrl_mode=ROTATE;//ƫ������ģʽ
			trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];//ƫ��������Դ�ں���˸���		
			steer_control(&turn_ctrl_pwm);
			speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���	
			//�����ٶ�
			speed_expect[0]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//��������ٶ�����
			speed_expect[1]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//�ұ������ٶ�����
			//�ٶȿ���
			speed_control_100hz(speed_ctrl_mode);			
		}
	}
}

