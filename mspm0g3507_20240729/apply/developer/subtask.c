#include "headfile.h"
#include "subtask.h"

#define SUBTASK_NUM 20
#define flight_subtask_delta 5//5ms



uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//�����������̼߳��������������ڿ���ÿ���������̵߳�ִ��
uint32_t flight_global_cnt[SUBTASK_NUM]={0}; //������������ȫ�ּ����������Խ��λ��ƫ�������ж��жϺ����Ƿ񵽴�
uint32_t execute_time_ms[SUBTASK_NUM]={0};//������������ִ��ʱ�䣬������������ĳ�����̵߳�ִ��ʱ��

void flight_subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
	}
}



void flight_subtask_1(void)//˳ʱ��ת90��
{
	static uint8_t n=0;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =90;//˳ʱ��90��	
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�	
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else//��������
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];		
	}
}

void flight_subtask_2(void)//��ʱ��ת90��
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =90;//��ʱ��90��
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�	
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else//��������
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];		
	}
}



//��30deg/s�Ľ��ٶ�˳ʱ��ת��3000ms����ɺ���
void flight_subtask_3(void)
{
	static uint8_t n=2;
	if(flight_subtask_cnt[n]==0)
	{
		
		trackless_output.yaw_ctrl_mode=CLOCKWISE_TURN;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =30;//��30deg/s�Ľ��ٶ�˳ʱ��ת��3000ms
		trackless_output.execution_time_ms=3000;//ִ��ʱ��
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE_TURN;
		trackless_output.yaw_outer_control_output  =0;
		
	  if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
}

//��30deg/s�Ľ��ٶ���ʱ��ת��3000ms
void flight_subtask_4(void)
{
	static uint8_t n=3;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =30;//��30deg/s�Ľ��ٶ�˳ʱ��ת��3000ms
		trackless_output.execution_time_ms=3000;//ִ��ʱ��
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		trackless_output.yaw_outer_control_output  =0;
		
	  if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
	}
}



void flight_subtask_5(void)
{
	static uint8_t n=4;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		speed_setup=40;//ǰ��
		if(rangefinder.distance<50)//ǰ������Ƚ�С������ת��
		{
			speed_setup=0;//ֹͣ
			flight_subtask_cnt[n]=1;	
		}			
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =90;//˳ʱ��90��
		
		speed_setup=0;//ֹͣ		
		flight_subtask_cnt[n]=2;		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE;
		trackless_output.yaw_outer_control_output  =0;
		
		if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=3;//ִ����Ϻ��л�����һ�׶�	
	}
	else if(flight_subtask_cnt[n]==3)//ת����Ϻ󣬼����ж�ǰ������
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		if(rangefinder.distance<50)//ǰ������Ƚ�С������ת��
		{
			flight_subtask_cnt[n]=1;//����ת��	
		}
		else
		{
			flight_subtask_cnt[n]=0;//�ָ�ǰ��	
		}
	}
	else//��������
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_ROLL];
		speed_setup=RC_Data.rc_rpyt[RC_PITCH];//�ٶ�������Դ�ڸ����˸���
	}
} 


#define start_point_precision_cm 0.5f
#define distance_precision_cm 1.0f
#define steer_execute_time_ms 500
#define steer_value_default 300
#define freedom_time_ms 5000					//�ڳ�������ͣ��ʱ��



_park_params park_params={
	._track_speed_cmps=track_speed_cmps_default,
	._start_point_adjust1=start_point_adjust1_default,
	._forward_distance_cm=forward_distance_cm_default,
	._backward_distance1_cm=backward_distance1_cm_default,
	._backward_distance2_cm=backward_distance2_cm_default,
	._out_forward_distance1_cm=out_forward_distance1_cm_default,
	._out_forward_distance2_cm=out_forward_distance2_cm_default,
	._start_point_adjust2=start_point_adjust2_default,
	._parallel_backward_distance1_cm=parallel_backward_distance1_cm_default,
	._parallel_backward_distance2_cm=parallel_backward_distance2_cm_default,
	._parallel_backward_distance3_cm=parallel_backward_distance3_cm_default
};


void auto_reverse_stall_park(void)
{
	static uint8_t n=5;
	static float steer_gradient_cnt=0;
	static float servo_ctrl_value=300;
	if(flight_subtask_cnt[n]==0)//��һ�׶�����Ѱ��
	{
		speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
		vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//�����ٶ�
		speed_expect[0]=park_params._track_speed_cmps;//��������ٶ�����
		speed_expect[1]=park_params._track_speed_cmps;//�ұ������ٶ�����
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);

		//�ж�;������������
		if(camera1.carpark_num==3)//������2
		{
			flight_subtask_cnt[n]++;
			//����ǰ��5cm
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._start_point_adjust1;			
		}
	}
	else if(flight_subtask_cnt[n]==1)//�Ӿ�ʶ�𵽳����������,ִ�м���ǰ�����ߺ���
	{
		if(park_params._start_point_adjust1>0)//�������ǰ������ʼ�����㣬�����Ѱ��
		{
			vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
		}
		else turn_ctrl_pwm=0;//��̥�������������
		
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);
		
		if(flight_global_cnt[n]<20)//����20������λ��ƫ���С,����Ϊλ�ÿ������
		{
			if(ABS(distance_ctrl.error)<start_point_precision_cm)	flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;			
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=-steer_value_default;
			steer_gradient_cnt=execute_time_ms[n];
		}
	}
	else if(flight_subtask_cnt[n]==2)//�ڶ��׶�ͣ����,�������
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._forward_distance_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==3)//�����׶�
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);
		
		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=steer_value_default;
			//�����ٶ�
			//speed_expect[0]=0;//��������ٶ�����
			//speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];	
		}
	}
	else	if(flight_subtask_cnt[n]==4)//���Ľ׶�
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//��0���䵽1
		float steer_gradient_value_n1_p1=(steer_gradient_value0_1-0.5f)/0.5f;//��-1���䵽1
		
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_n1_p1);
		
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
	  //�����ٶ�
		speed_expect[0]=0;//��������ٶ�����
		speed_expect[1]=0;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._backward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==5)//����׶�
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];		
		}
	}
	else	if(flight_subtask_cnt[n]==6)//�����׶�
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//��0���䵽1
		float steer_gradient_value_1_0=1.0f-steer_gradient_value0_1;//��1���䵽0
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_1_0);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._backward_distance2_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==7)//����׶�
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=freedom_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=0;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����		
		}
	}
	else if(flight_subtask_cnt[n]==8)//�ڳ�βͣ��5S
	{
		//ת�����
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._out_forward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==9)//������ǰ��
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=300;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����		
		}	
	}
	else if(flight_subtask_cnt[n]==10)
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		speed_control_100hz(speed_ctrl_mode);
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._out_forward_distance2_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==11)//����ת��
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=0;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����		
		}	
	}
	else if(flight_subtask_cnt[n]==12)//����������������һ����
	{
		for(uint16_t i=0;i<4;i++)
		{
			camera1.carpark_flag[0][i]=0;
			camera1.carpark_flag[1][i]=0;
		}
		camera1.carpark_num=0;
		sdk_work_mode+=1;
	}
	else
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);	
	}
}



void auto_parallel_park(void)
{
	static uint8_t n=6;	
	static float steer_gradient_cnt=0;
	static float servo_ctrl_value=300;
	
	if(flight_subtask_cnt[n]==0)//��һ�׶�����Ѱ��
	{
		speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
		vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//�����ٶ�
		speed_expect[0]=park_params._track_speed_cmps;//��������ٶ�����
		speed_expect[1]=park_params._track_speed_cmps;//�ұ������ٶ�����
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);

		//�ж�;������������
		if(camera1.carpark_num==3)//������2
		{
			flight_subtask_cnt[n]++;
			//����ǰ��5cm
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._start_point_adjust2;			
		}
	}
	else if(flight_subtask_cnt[n]==1)//ִ�м���ǰ��10cm
	{
		if(park_params._start_point_adjust2>0)//�������ǰ������ʼ�����㣬�����Ѱ��
		{
			vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
		}
		else turn_ctrl_pwm=0;//��̥�������������
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);
		
		if(flight_global_cnt[n]<20)
		{
			if(ABS(distance_ctrl.error)<start_point_precision_cm)	flight_global_cnt[n]++;	
			else flight_global_cnt[n]/=2;			
		}
		else 
		{
			flight_global_cnt[n]=0;
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];
		}
	}
	else if(flight_subtask_cnt[n]==2)//�ڶ��׶�ɲ����ת��
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._parallel_backward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==3)//�����׶�
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];	
		}
	}
	else if(flight_subtask_cnt[n]==4)//��ͷ����
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//��0���䵽1
		float steer_gradient_value_1_0=1.0f-steer_gradient_value0_1;//��1���䵽0
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_1_0);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._parallel_backward_distance2_cm;
		}			
	}
	else if(flight_subtask_cnt[n]==5)//��ֱ����
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=-steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];	
		}	
	}
	else if(flight_subtask_cnt[n]==6)//����������
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance-park_params._parallel_backward_distance3_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==7)//���������򲢺���
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=freedom_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=0;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
		}		
	}
	else if(flight_subtask_cnt[n]==8)//����ֱ�ӻ���,ԭ�صȴ�5s
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		speed_setup=0;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=-steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];
		}	
	}
	else if(flight_subtask_cnt[n]==9)//��̥�����׼������
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);	
		//�����ٶ�
		speed_expect[0]=0;//��������ٶ�����
		speed_expect[1]=0;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);	
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=-steer_value_default;	
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._parallel_backward_distance3_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==10)//����������ǰ��
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=-steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];
		}		
	}
	else if(flight_subtask_cnt[n]==11)//��̥�𽥻��������ǰ��
	{
		float steer_gradient_value0_1=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);//��0���䵽1
		float steer_gradient_value_1_0=1.0f-steer_gradient_value0_1;//��1���䵽0
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value_1_0);
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._parallel_backward_distance2_cm;
		}			
	}
	else if(flight_subtask_cnt[n]==12)//���������ǰ��
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=steer_value_default;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
			steer_gradient_cnt=execute_time_ms[n];
		}		
	}
	else if(flight_subtask_cnt[n]==13)//��̥���Ҵ�
	{
		float steer_gradient_value=(float)((steer_gradient_cnt-execute_time_ms[n])/steer_gradient_cnt);
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value*steer_gradient_value);	
		//�����ٶ�
		speed_expect[0]=0;//��������ٶ�����
		speed_expect[1]=0;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);	
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=steer_execute_time_ms/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=steer_value_default;	
			distance_ctrl.expect=smartcar_imu.state_estimation.distance+park_params._parallel_backward_distance1_cm;
		}		
	}
	else if(flight_subtask_cnt[n]==14)//��̥���Ҵ�����ǰ��
	{
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
		//�������
		distance_control();
		speed_setup=distance_ctrl.output;
		//�����ٶ�
		speed_expect[0]=speed_setup;//��������ٶ�����
		speed_expect[1]=speed_setup;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);

		if(ABS(distance_ctrl.error)<distance_precision_cm)
		{
			flight_subtask_cnt[n]++;
			execute_time_ms[n]=3000/flight_subtask_delta;//������ִ��ʱ��;
			servo_ctrl_value=0;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
		}		
	}
	else if(flight_subtask_cnt[n]==15)//����Ѳ��
	{
		speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
		vision_turn_control_50hz(&turn_ctrl_pwm);//����OPENMV�Ӿ������ת�����
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+turn_ctrl_pwm);	
		//�����ٶ�
		speed_expect[0]=park_params._track_speed_cmps;//��������ٶ�����
		speed_expect[1]=park_params._track_speed_cmps;//�ұ������ٶ�����
		//�ٶȿ���
		speed_control_100hz(speed_ctrl_mode);	
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]++;
			servo_ctrl_value=0;
			//�����ٶ�
			speed_expect[0]=0;//��������ٶ�����
			speed_expect[1]=0;//�ұ������ٶ�����
		}	
	}
	else//Ѳ�߽�����ֹͣ
	{
		speed_ctrl_mode=1;//�ٶȿ��Ʒ�ʽΪ���ֵ�������
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		//�ٶȿ���
		speed_expect[0]=0;//��������ٶ�����
		speed_expect[1]=0;//�ұ������ٶ�����
		speed_control_100hz(speed_ctrl_mode);		
	}
}


